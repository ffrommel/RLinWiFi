#ifndef SCENARIO_H
#define SCENARIO_H

#include <fstream>
#include <string>
#include <math.h>
#include <ctime>
#include <iomanip>
#include <deque>
#include <algorithm>

using namespace std;
using namespace ns3;

int nWifi = 5;
int nAx = 2;
bool uplink = true;
bool udp = true;
int headerSize;

uint64_t g_rxDataPktNum = 0;
vector<int> effective_stas_legacy(51, 0);

void udpDataPacketReceived(Ptr<const Packet> packet)
{
    //int id = std::stoi(str_id);
    g_rxDataPktNum++;
}

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
    std::vector<double> start_times;
    std::vector<double> end_times;
    int history_length;

    void installUDPTrafficGenerator(ns3::Ptr<ns3::Node> fromNode, ns3::Ptr<ns3::Node> toNode, int port, int payloadSize, std::string offeredLoad, double startTime,
        double endTime);
    void installTCPTrafficGenerator(ns3::Ptr<ns3::Node> fromNode, ns3::Ptr<ns3::Node> toNode, ns3::Ipv4Address rxAddr, int port, int payloadSize, double startTime,
        double endTime);

  public:
    Scenario(int nWifim, int nAxm, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer stasInterface, Ipv4InterfaceContainer apInterface, int port,
        std::string dataRate, int history_length);
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
    int history_length;
    std::string dataRate; // Used only with UDP traffic

  public:
    ScenarioFactory(int nWifim, int nAxm, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer stasInterface, Ipv4InterfaceContainer apInterface,
        int port, std::string dataRate, int history_length)
    {
        this->nWifim = nWifim;
        this->nAxm = nAxm;
        this->staNodes = staNodes;
        this->apNode = apNode;
        this->stasInterface = stasInterface;
        this->apInterface = apInterface;
        this->port = port;
        this->dataRate = dataRate;
        this->history_length = history_length;
    }

    Scenario *getScenario(std::string scenario)
    {
        Scenario *wifiScenario;
        if (scenario == "basic")
        {
            wifiScenario = new BasicScenario(this->nWifim, this->nAxm, this->staNodes, this->apNode, this->stasInterface, this->apInterface, this->port, this->dataRate,
                this->history_length);
        }
        else if (scenario == "convergence")
        {
            wifiScenario = new ConvergenceScenario(this->nWifim, this->nAxm, this->staNodes, this->apNode, this->stasInterface, this->apInterface, this->port,
                this->dataRate, this->history_length);
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
    std::string dataRate, int history_length)
{
    this->nWifim = nWifim;
    this->nAxm = nAxm;
    this->staNodes = staNodes;
    this->apNode = apNode;
    this->stasInterface = stasInterface;
    this->apInterface = apInterface;
    this->port = port;
    this->dataRate = dataRate;
    this->history_length = history_length;
}

int Scenario::getActiveStationCount(double time)
{
    int res=0;
    for(uint i=0; i<start_times.size(); i++)
        if(start_times.at(i)<time && time<end_times.at(i))
            res++;
    return res;
}

float Scenario::getStationUptime(int id, double time)
{
    return time - start_times.at(id);
}

void Scenario::installUDPTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, int payloadSize, string offeredLoad, double startTime, double endTime)
{
    start_times.push_back(startTime);
    end_times.push_back(endTime);
    //static double d = 0;

    double min = 0;
    double max = 1;
    Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable>();
    fuzz->SetAttribute("Min", DoubleValue(min));
    fuzz->SetAttribute("Max", DoubleValue(max));

    Ptr<Ipv4> ipv4 = toNode->GetObject<Ipv4>();           // Get Ipv4 instance of the node
    Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.

    ApplicationContainer sourceApplications, sinkApplications;

    uint8_t tosValue = 0x70; //AC_BE

    InetSocketAddress sinkSocket(addr, port);
    sinkSocket.SetTos(tosValue);
    OnOffHelper onOffHelper("ns3::UdpSocketFactory", sinkSocket);
    onOffHelper.SetConstantRate(DataRate(offeredLoad + "Mbps"), payloadSize);
    sourceApplications.Add(onOffHelper.Install(fromNode));
    //sourceApplications.StartWithJitter(Seconds(startTime), fuzz);
    sourceApplications.Start(Seconds(startTime));
    //d = d + 0.5;
    sourceApplications.Stop(Seconds(endTime));

    UdpServerHelper sink(port);
    sinkApplications = sink.Install(toNode);
    sinkApplications.StartWithJitter(Seconds(startTime), fuzz);
    sinkApplications.Stop(Seconds(endTime));

    // Trace for throughput calculation -- TCP
    Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(sinkApplications.Get(0));
    //std::string ss = std::to_string(fromNode->GetId());
    //udpServer->TraceConnect("Rx", ss, MakeCallback(&udpPacketReceived));
    udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&udpDataPacketReceived));
}

void Scenario::installTCPTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, Ipv4Address rxAddr, int port, int payloadSize, double startTime, double endTime)
{
    start_times.push_back(startTime);
    end_times.push_back(endTime);

    // Set payload size
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
    // Set sender and receiver buffer size
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (1 << 20));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (1 << 20));
    // Set default initial congestion window
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));


    // Create a BulkSendApplication and install it on fromNode
    BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (rxAddr, port));
    // Set the amount of data to send in bytes. Zero is unlimited.
    source.SetAttribute ("MaxBytes", UintegerValue (0));
    ApplicationContainer sourceApps = source.Install (fromNode);
    sourceApps.Start (Seconds (startTime + 1));
    sourceApps.Stop (Seconds (endTime));

    // Create a PacketSinkApplication and install it on toNode
    PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = sink.Install (toNode);
    sinkApps.Start (Seconds (startTime));
    sinkApps.Stop (Seconds (endTime));
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

    // Useful IPs to analyze PCAPs
    NS_LOG_UNCOND("\nSTA 0 IP: " << this->stasInterface.GetAddress(0));
    NS_LOG_UNCOND("\nAP IP: " << this->apInterface.GetAddress(0));

    // Install 'nAxm' flows in STA 0 (802.11ax devices)
    for (int i = 0; i < this->nAxm; ++i)
    {
        if (uplink)
        {
            if (udp)
            {
                installUDPTrafficGenerator(this->staNodes.Get(0), this->apNode.Get(0), this->port++, payloadSize, this->dataRate, 0.0, simulationTime + 2 +
                    envStepTime*history_length);
            }
            else // TCP
            {
                installTCPTrafficGenerator(this->staNodes.Get(0), this->apNode.Get(0), this->apInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                    simulationTime + 2 + envStepTime*history_length);
            }
        }
        else // Downlink
        {
            NS_LOG_UNCOND("EXIT! Coexistence ax - legacy not implemented in DL");
            exit(0);
        }
    }
    // Install 1 flow in the rest of STAs (legacy devices)
    for (int i = 1; i < (this->nWifim - this->nAxm + 1); ++i)
    {
        if (uplink)
        {
            if (udp)
            {
                installUDPTrafficGenerator(this->staNodes.Get(i), this->apNode.Get(0), this->port++, payloadSize, this->dataRate, 0.0, simulationTime + 2 +
                    envStepTime*history_length);
            }
            else // TCP
            {
                installTCPTrafficGenerator(this->staNodes.Get(i), this->apNode.Get(0), this->apInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                    simulationTime + 2 + envStepTime*history_length);
            }
        }
        else // Downlink
        {
            NS_LOG_UNCOND("EXIT! Coexistence ax - legacy not implemented in DL");
            exit(0);
        }
    }
}

void ConvergenceScenario::updateMaxAmpduSize (int value)
{
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (this->staNodes.Get(0)->GetDevice(0));
    wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (value));
}

void ConvergenceScenario::installScenario(double simulationTime, double envStepTime, bool uplink, bool udp, int payloadSize)
{
    NS_LOG_UNCOND("EXIT! Coexistence ax - legacy not implemented for convergence scenario");
    exit(0);
/*
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
*/
}
#endif
