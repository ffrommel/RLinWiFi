#ifndef SCENARIO_H
#define SCENARIO_H

#include <fstream>
#include <string>
#include <math.h>
#include <ctime>   //timestampi
#include <iomanip> // put_time
#include <deque>
#include <algorithm>

using namespace std;
using namespace ns3;

ApplicationContainer serversApp = ApplicationContainer();
ApplicationContainer clientsApp = ApplicationContainer();
int headerSize;

class Scenario
{
  protected:
    int nWifim;
    int nAxm;
    NodeContainer staNodes;
    NodeContainer apNode;
    Ipv4InterfaceContainer stasInterface;
    Ipv4InterfaceContainer apInterface;
    int port;
    std::string dataRate; // Used only with UDP traffic
    std::vector<double> startTimes;
    std::vector<double> endTimes;
    int historyLength;

    void generateUdpTraffic (double startTime,
                             double endTime,
                             int payloadSize,
                             ns3::Ptr<ns3::Node> srvNode,
                             ns3::Ptr<ns3::Node> cliNode,
                             int port);

    void generateTcpTraffic (double startTime,
                             double endTime,
                             int payloadSize,
                             ns3::Ptr<ns3::Node> srvNode,
                             ns3::Ptr<ns3::Node> cliNode,
                             ns3::Ipv4Address address,
                             int port);

  public:
    Scenario(int nWifim, int nAxm, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer stasInterface, Ipv4InterfaceContainer apInterface, int port,
        std::string dataRate, int historyLength);
    virtual void installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize) = 0;
    void PopulateARPcache();
    int getActiveStationCount(double time);
    float getStationUptime(int id, double time);
};

class BasicScenario : public Scenario
{
    using Scenario::Scenario;

  public:
    void installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize) override;
};

class ConvergenceScenario : public Scenario
{
    using Scenario::Scenario;
    void updateMaxAmpduSize (int value);

  public:
    void installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize) override;
};

class ScenarioFactory
{
  private:
    int nWifim;
    int nAxm;
    NodeContainer staNodes;
    NodeContainer apNode;
    Ipv4InterfaceContainer stasInterface;
    Ipv4InterfaceContainer apInterface;
    int port;
    int historyLength;
    std::string dataRate; // Used only with UDP traffic

  public:
    ScenarioFactory(int nWifim, int nAxm, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer stasInterface, Ipv4InterfaceContainer apInterface, 
        int port, std::string dataRate, int historyLength)
    {
        this->nWifim = nWifim;
        this->nAxm = nAxm;
        this->staNodes = staNodes;
        this->apNode = apNode;
        this->stasInterface = stasInterface;
        this->apInterface = apInterface;
        this->port = port;
        this->dataRate = dataRate;
        this->historyLength = historyLength;
    }

    Scenario *getScenario(std::string scenario)
    {
        Scenario *wifiScenario;
        if (scenario == "basic")
        {
            wifiScenario = new BasicScenario(this->nWifim, this->nAxm, this->staNodes, this->apNode, this->stasInterface, this->apInterface, this->port, this->dataRate,
                this->historyLength);
        }
        else if (scenario == "convergence")
        {
            wifiScenario = new ConvergenceScenario(this->nWifim, this->nAxm, this->staNodes, this->apNode, this->stasInterface, this->apInterface, this->port,
                this->dataRate, this->historyLength);
        }
        else
        {
            std::cout << "Unsupported scenario" << endl;
            exit(0);
        }
        return wifiScenario;
    }
};

Scenario::Scenario(int nWifim, int nAxm, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer stasInterface, Ipv4InterfaceContainer apInterface, int port,
    std::string dataRate, int historyLength)
{
    this->nWifim = nWifim;
    this->nAxm = nAxm;
    this->staNodes = staNodes;
    this->apNode = apNode;
    this->stasInterface = stasInterface;
    this->apInterface = apInterface;
    this->port = port;
    this->dataRate = dataRate;
    this->historyLength = historyLength;
}

int Scenario::getActiveStationCount(double time)
{
    int res=0;
    for(uint i=0; i<startTimes.size(); i++)
        if(startTimes.at(i)<time && time<endTimes.at(i))
            res++;
    return res;
}

float Scenario::getStationUptime(int id, double time)
{
    return time - startTimes.at(id);
}

// UDP traffic: cli -> srv
void Scenario::generateUdpTraffic (double startTime, double endTime, int payloadSize, Ptr<Node> srvNode, Ptr<Node> cliNode, int port)
{
    startTimes.push_back(startTime);
    endTimes.push_back(endTime);

    Ptr<Ipv4> ipv4 = srvNode->GetObject<Ipv4>();          // Get Ipv4 instance of the node
    Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.

    uint8_t tosValue = 0x70; //AC_BE

    InetSocketAddress sinkSocket(addr, port);
    sinkSocket.SetTos(tosValue);
    OnOffHelper onOffHelper("ns3::UdpSocketFactory", sinkSocket);
    onOffHelper.SetConstantRate(DataRate(dataRate + "Mbps"), payloadSize);

    clientsApp.Add (onOffHelper.Install(cliNode));

    UdpServerHelper sink(port);
    serversApp.Add (sink.Install(srvNode));
}

// TCP traffic: cli -> srv
void Scenario::generateTcpTraffic (double startTime, double endTime, int payloadSize, Ptr<Node> srvNode, Ptr<Node> cliNode, Ipv4Address address, int port)
{
        startTimes.push_back(startTime);
        endTimes.push_back(endTime);

        uint8_t tosValue = 0x70; //AC_BE

        InetSocketAddress sinkSocket(Ipv4Address::GetAny (), port);
        sinkSocket.SetTos(tosValue);
        Address localAddress (sinkSocket);
        PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", localAddress);
        serversApp.Add (packetSinkHelper.Install (srvNode));

        Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
        OnOffHelper onoff ("ns3::TcpSocketFactory", Ipv4Address::GetAny ());
        onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));

        AddressValue remoteAddress (InetSocketAddress (address, port));
        onoff.SetAttribute ("Remote", remoteAddress);
        clientsApp.Add (onoff.Install (cliNode));
}

void Scenario::PopulateARPcache()
{
    Ptr<ArpCache> arp = CreateObject<ArpCache>();
    arp->SetAliveTimeout(Seconds(3600 * 24 * 365));

    for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i)
    {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol>();
        NS_ASSERT(ip != 0);
        ObjectVectorValue interfaces;
        ip->GetAttribute("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin(); j != interfaces.End(); j++)
        {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface>();
            NS_ASSERT(ipIface != 0);
            Ptr<NetDevice> device = ipIface->GetDevice();
            NS_ASSERT(device != 0);
            Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress());

            for (uint32_t k = 0; k < ipIface->GetNAddresses(); k++)
            {
                Ipv4Address ipAddr = ipIface->GetAddress(k).GetLocal();
                if (ipAddr == Ipv4Address::GetLoopback())
                    continue;

                ArpCache::Entry *entry = arp->Add(ipAddr);
                Ipv4Header ipv4Hdr;
                ipv4Hdr.SetDestination(ipAddr);
                Ptr<Packet> p = Create<Packet>(100);
                entry->MarkWaitReply(ArpCache::Ipv4PayloadHeaderPair(p, ipv4Hdr));
                entry->MarkAlive(addr);
            }
        }
    }

    for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i)
    {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol>();
        NS_ASSERT(ip != 0);
        ObjectVectorValue interfaces;
        ip->GetAttribute("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin(); j != interfaces.End(); j++)
        {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface>();
            ipIface->SetAttribute("ArpCache", PointerValue(arp));
        }
    }
}

void BasicScenario::installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize)
{
    // Header size (extracted from PCAP analysis)
    headerSize = udp ? 114 : 138; // bytes

    // Enable A-MPDU for STA 0
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (staNodes.Get(0)->GetDevice(0));
    wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (this->nAxm * (headerSize + payloadSize))); // if nAxm = 0, A-MPDU aggregation get disabled

    // Install 'nAxm' flows in STA 0 (802.11ax devices)
    for (int i = 0; i < this->nAxm; ++i)
    {
        if (uplink)
        {
            if (udp)
            {
                generateUdpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->apNode.Get(0), this->staNodes.Get(0), this->port++);
            }
            else // TCP
            {
                generateTcpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->apNode.Get(0), this->staNodes.Get(0),
                    this->apInterface.GetAddress(0), this->port++); // Receptor's address
            }
        }
        else // Downlink
        {
            if (udp)
            {
                generateUdpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->staNodes.Get(0), this->apNode.Get(0), this->port++);
            }
            else // TCP
            {
                generateTcpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->staNodes.Get(0), this->apNode.Get(0),
                    this->stasInterface.GetAddress(i), this->port++); // Receptor's address
            }
        }
    }
    // Install 1 flow in the rest of STAs (legacy devices)
    for (int i = 1; i < (this->nWifim - this->nAxm + 1); ++i)
    {
        if (uplink)
        {
            if (udp)
            {
                generateUdpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->apNode.Get(0), this->staNodes.Get(i), this->port++);
            }
            else // TCP
            {
                generateTcpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->apNode.Get(0), this->staNodes.Get(i),
                    this->apInterface.GetAddress(0), this->port++); // Receptor's address
            }
        }
        else // Downlink
        {
            if (udp)
            {
                generateUdpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->staNodes.Get(i), this->apNode.Get(0), this->port++);
            }
            else // TCP
            {
                generateTcpTraffic(0.0, simulationTime + 2 + envStepTime*historyLength, payloadSize, this->staNodes.Get(i), this->apNode.Get(0),
                    this->stasInterface.GetAddress(i), this->port++); // Receptor's address
            }
        }
    }
    // Start all the flows
    if (udp && !uplink) // UDP downlink
    {
        for (int i = 0; i < this->nWifim; ++i)
        {
            ApplicationContainer srv_aux = serversApp.Get (i); // First 'nAxm' flows of STA 0 and the rest of STAs > 0
            ApplicationContainer cli_aux = clientsApp.Get (i);
            srv_aux.Start (Seconds (i*0.1));
            srv_aux.Stop (Seconds (simulationTime + 2 + envStepTime*historyLength));
            cli_aux.Start (Seconds (i*0.1));
            cli_aux.Stop (Seconds (simulationTime + 2 + envStepTime*historyLength));
        }
    }
    else // UDP uplink, TCP uplink or TCP downlink
    {
        double min = 0.0;
        double max = uplink ? 0.0 : 1.0;
        Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable>();
        fuzz->SetAttribute("Min", DoubleValue(min));
        fuzz->SetAttribute("Max", DoubleValue(max));
        serversApp.Start (Seconds (0.0));
        serversApp.Stop (Seconds (simulationTime + 2 + envStepTime*historyLength));
        clientsApp.StartWithJitter (Seconds (max), fuzz);
        clientsApp.Stop (Seconds (simulationTime + 2 + envStepTime*historyLength));
    }
}

void ConvergenceScenario::updateMaxAmpduSize (int value)
{
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (this->staNodes.Get(0)->GetDevice(0));
    wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (value));
}

void ConvergenceScenario::installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize)
{
    float delta = simulationTime/(this->nWifim-4);
    float delay = historyLength*envStepTime;

    double min = 0.0;
    double max = uplink ? 0.0 : 1.0;
    Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable>();
    fuzz->SetAttribute("Min", DoubleValue(min));
    fuzz->SetAttribute("Max", DoubleValue(max));

    int n = 0; // First 802.11ax devices to consider
    int j = 0, k = 0; // STA indexes
    int start = 0, end = 0; // Start and end values of the second 'for'
    int c = 1; // Counter for the rest of 802.11ax devices to consider
    int value = 0; // Max A-MPDU size value

    // Header size (extracted from PCAP analysis)
    headerSize = udp ? 114 : 138; // bytes

    if (this->nWifim > 5)
    {
        // Enable A-MPDU for STA 0
        n = this->nAxm < 5 ? this->nAxm : 5; // Up to 5 flows at this time
        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (staNodes.Get(0)->GetDevice(0));
        wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (n * (headerSize + payloadSize))); // if nAxm = 0, A-MPDU aggregation get disabled

        // Install the first 5 flows, starting with all 802.11ax devices
        for (int i = 0; i < 5; ++i)
        {
            // Calculate STA index
            if (this->nAxm > 0)
            {
                j = i < this->nAxm ? 0 : i - this->nAxm + 1;
            } else {
                j = i;
            }
            if (uplink)
            {
                if (udp)
                {
                    generateUdpTraffic(0.0, simulationTime + 2 + delay, payloadSize, this->apNode.Get(0), this->staNodes.Get(j), this->port++);
                }
                else // TCP
                {
                    generateTcpTraffic(0.0, simulationTime + 2 + delay, payloadSize, this->apNode.Get(0), this->staNodes.Get(j),
                        this->apInterface.GetAddress(0), this->port++); // Receptor's address
                }
            }
            else // Downlink
            {
                if (udp)
                {
                    generateUdpTraffic(0.0, simulationTime + 2 + delay, payloadSize, this->staNodes.Get(j), this->apNode.Get(0), this->port++);
                }
                else // TCP
                {
                    generateTcpTraffic(0.0, simulationTime + 2 + delay, payloadSize, this->staNodes.Get(j), this->apNode.Get(0),
                        this->stasInterface.GetAddress(j), this->port++); // Receptor's address
                }
            }
        }
        // Start the first 5 flows
        if (udp && !uplink) // UDP downlink
        {
            for (int i = 0; i < 5; ++i)
            {
                ApplicationContainer srv_aux = serversApp.Get (i);
                ApplicationContainer cli_aux = clientsApp.Get (i);
                srv_aux.Start (Seconds (i*0.1));
                srv_aux.Stop (Seconds (simulationTime + 2 + delay));
                cli_aux.Start (Seconds (i*0.1));
                cli_aux.Stop (Seconds (simulationTime + 2 + delay));
            }
        }
        else // UDP uplink, TCP uplink or TCP downlink
        {
            // Get first 5 apps
            ApplicationContainer srv_aux, cli_aux;
            for (int i = 0; i < 5; ++i)
            {
                srv_aux.Add (serversApp.Get (i));
                cli_aux.Add (clientsApp.Get (i));
            }
            srv_aux.Start (Seconds (0.0));
            srv_aux.Stop (Seconds (simulationTime + 2 + delay));
            cli_aux.StartWithJitter (Seconds (max), fuzz);
            cli_aux.Stop (Seconds (simulationTime + 2 + delay));
        }
        // Install the rest of the flows, starting with all 802.11ax devices
        start = this->nAxm < 5 ? j + 1 : 5;
        end = (this->nAxm > 0 && this->nAxm < 5) ? this->nWifim - this->nAxm + j : this->nWifim;
        for (int i = start; i < end; ++i)
        {
            // Calculate STA index
            if (this->nAxm >= 5)
            {
                k = i < this->nAxm ? 0 : i - this->nAxm + 1;
            } else {
                k = i;
            }
            // Update A-MPDU max for STA 0 in each step until all 802.11ax devices are in the network
            if (this->nAxm > 5 && i < this->nAxm)
            {
                value = (5 + c) * (headerSize + payloadSize);
                Simulator::Schedule(Seconds(delay + (i - start + 1) * delta), &ConvergenceScenario::updateMaxAmpduSize, this, value);
                c++;
            }
            if (uplink)
            {
                if (udp)
                {
                    generateUdpTraffic(delay + (i - start + 1) * delta, simulationTime + 2 + delay, payloadSize, this->apNode.Get(0), this->staNodes.Get(k), this->port++);
                }
                else // TCP
                {
                    generateTcpTraffic(delay + (i - start + 1) * delta, simulationTime + 2 + delay, payloadSize, this->apNode.Get(0), this->staNodes.Get(k),
                        this->apInterface.GetAddress(0), this->port++); // Receptor's address
                }
            }
            else // Downlink
            {
            if (udp)
                {
                    generateUdpTraffic(delay + (i - start + 1) * delta, simulationTime + 2 + delay, payloadSize, this->staNodes.Get(k), this->apNode.Get(0), this->port++);
                }
                else // TCP
                {
                    generateTcpTraffic(delay + (i - start + 1) * delta, simulationTime + 2 + delay, payloadSize, this->staNodes.Get(k), this->apNode.Get(0),
                        this->stasInterface.GetAddress(k), this->port++); // Receptor's address
                }
            }
        }
        // Start the rest of the flows
        double delay_2 = 0.0;
        for (int i = 5; i < this->nWifim; ++i)
        {
            ApplicationContainer srv_aux = serversApp.Get (i);
            ApplicationContainer cli_aux = clientsApp.Get (i);
            if (udp && !uplink) // UDP downlink
            {
                delay_2 = i*0.1;
                max = 0;
                fuzz->SetAttribute("Max", DoubleValue(max)); // Remove jitter
            }
            srv_aux.Start (Seconds (delay + (i - 4) * delta + delay_2));
            srv_aux.Stop (Seconds (simulationTime + 2 + delay));
            cli_aux.StartWithJitter (Seconds (delay + (i - 4) * delta + delay_2 + max), fuzz);
            cli_aux.Stop (Seconds (simulationTime + 2 + delay));
        }
    }
    else
    {
        std::cout << "Not enough Wi-Fi stations to support the convergence scenario." << endl;
        exit(0);
    }
}
#endif
