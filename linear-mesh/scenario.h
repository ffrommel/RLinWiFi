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

string filename[4] = {"tx_pkts.csv", "rx_pkts.csv", "cw_values.csv", "threshold.csv"};
ofstream outputFile[4];

int nWifi = 5;
bool udp = true;                // true: UDP, false: TCP
int direction = 0;              // 0: UL, 1: DL, 2: UL+DL
bool mixedScenario = false;

uint64_t g_rxDataPktNum = 0;
vector<int> effective_stas(51, 0);

void udpDataPacketReceived(Ptr<const Packet> packet)
{
    //int id = std::stoi(str_id);
    g_rxDataPktNum++;
}

class Scenario
{
  protected:
    int nWifim;
    NodeContainer wifiStaNode;
    NodeContainer wifiApNode;
    Ipv4InterfaceContainer staNodeInterface;
    Ipv4InterfaceContainer apNodeInterface;
    int port;
    std::string offeredLoad;
    std::vector<double> start_times;
    std::vector<double> end_times;
    int history_length;

    void installUDPTrafficGenerator(ns3::Ptr<ns3::Node> fromNode, ns3::Ptr<ns3::Node> toNode, int port, int payloadSize, std::string offeredLoad, double startTime,
        double endTime);
    void installTCPTrafficGenerator(ns3::Ptr<ns3::Node> fromNode, ns3::Ptr<ns3::Node> toNode, ns3::Ipv4Address rxAddr, int port, int payloadSize, double startTime,
        double endTime);

  public:
    Scenario(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, Ipv4InterfaceContainer staNodeInterface, Ipv4InterfaceContainer apNodeInterface,
        int port, std::string offeredLoad, int history_length);
    virtual void installScenario(double simulationTime, double envStepTime, bool udp, int direction, int payloadSize) = 0;
    void PopulateARPcache();
    int getActiveStationCount(double time);
    float getStationUptime(int id, double time);
};

class BasicScenario : public Scenario
{
    using Scenario::Scenario;

  public:
    void installScenario(double simulationTime, double envStepTime, bool udp, int direction, int payloadSize) override;
};

class ConvergenceScenario : public Scenario
{
    using Scenario::Scenario;

  public:
    void installScenario(double simulationTime, double envStepTime, bool udp, int direction, int payloadSize) override;
};

class ScenarioFactory
{
  private:
    int nWifim;
    NodeContainer wifiStaNode;
    NodeContainer wifiApNode;
    Ipv4InterfaceContainer staNodeInterface;
    Ipv4InterfaceContainer apNodeInterface;
    int port;
    int history_length;
    std::string offeredLoad;

  public:
    ScenarioFactory(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, Ipv4InterfaceContainer staNodeInterface, Ipv4InterfaceContainer apNodeInterface,
        int port, std::string offeredLoad, int history_length)
    {
        this->nWifim = nWifim;
        this->wifiStaNode = wifiStaNode;
        this->wifiApNode = wifiApNode;
        this->staNodeInterface = staNodeInterface;
        this->apNodeInterface = apNodeInterface;
        this->port = port;
        this->offeredLoad = offeredLoad;
        this->history_length = history_length;
    }

    Scenario *getScenario(std::string scenario)
    {
        Scenario *wifiScenario;
        if (scenario == "basic")
        {
            wifiScenario = new BasicScenario(this->nWifim, this->wifiStaNode, this->wifiApNode, this->staNodeInterface, this->apNodeInterface, this->port,
                this->offeredLoad, this->history_length);
        }
        else if (scenario == "convergence")
        {
            wifiScenario = new ConvergenceScenario(this->nWifim, this->wifiStaNode, this->wifiApNode, this->staNodeInterface, this->apNodeInterface, this->port,
                this->offeredLoad, this->history_length);
        }
        else
        {
            std::cout << "Unsupported scenario" << endl;
            exit(0);
        }
        return wifiScenario;
    }
};

Scenario::Scenario(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, Ipv4InterfaceContainer staNodeInterface, Ipv4InterfaceContainer apNodeInterface,
    int port, std::string offeredLoad, int history_length)
{
    this->nWifim = nWifim;
    this->wifiStaNode = wifiStaNode;
    this->wifiApNode = wifiApNode;
    this->staNodeInterface = staNodeInterface;
    this->apNodeInterface = apNodeInterface;
    this->port = port;
    this->offeredLoad = offeredLoad;
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
    double max;
    if (direction == 2) // UL+DL
        max = (fromNode->GetId() == (uint32_t) nWifi) ? 5.0 : 0.0; // Random in DL direction
    else // UL or DL
        max = 1.0;
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
    sinkApplications.Start(Seconds(startTime));
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

void BasicScenario::installScenario(double simulationTime, double envStepTime, bool udp, int direction, int payloadSize)
{
    string offeredLoad = this->offeredLoad;
    double startUdp = 0.0; // s
    for (int i = 0; i < this->nWifim; ++i)
    {
/*
        if (i < 10)
        {
            // TCP UL+DL (#STAs = 10) from 0 s to simulationTime/2 s
            // UL
            installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, 1460, 0.0, 30);
            // DL
            installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, 1460, 0.0, 30);
        }
        else
        {
            // TCP UL+DL (#STAs = 25) from simulationTime/2 s to simulationTime s
            // UL
            installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, 1460, 35,
                simulationTime + 2 + envStepTime*history_length);
            // DL
            installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, 1460, 35,
                simulationTime + 2 + envStepTime*history_length);
        }
*/
        // Force mixed scenario
        if (mixedScenario)
        {
            if (i < 5)
            {
                udp = true;
                direction = 0;
                payloadSize = 1472;  // bytes
                offeredLoad = "1.5"; // Mbps
                startUdp = 60.0;     // s
            }
            else
            {
                udp = false;
                payloadSize = 1460; // bytes
                direction = 2;
            }
        }
        if (udp)
        {
            switch(direction)
            {
                case 0: // UL
                    installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, offeredLoad, startUdp, simulationTime + 2 +
                        envStepTime*history_length);
                    break;
                case 1: // DL
                    installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, offeredLoad, startUdp, simulationTime + 2 +
                        envStepTime*history_length);
                    break;
                case 2: // UL+DL
                    installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, offeredLoad, startUdp, simulationTime + 2 +
                        envStepTime*history_length);
                    installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, offeredLoad, startUdp, simulationTime + 2 +
                        envStepTime*history_length);
                    break;
            }
        }
        else // TCP
        {
            switch(direction)
            {
                case 0: // UL
                    installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                        simulationTime + 2 + envStepTime*history_length);
                    break;
                case 1: // DL
                    installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize, 0.0,
                        simulationTime + 2 + envStepTime*history_length);
                    break;
                case 2: // UL+DL
                    installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                        simulationTime + 2 + envStepTime*history_length);
                    installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize, 0.0,
                        simulationTime + 2 + envStepTime*history_length);
                    break;
            }
        }

    }
}

void ConvergenceScenario::installScenario(double simulationTime, double envStepTime, bool udp, int direction, int payloadSize)
{
    float delta = simulationTime/(this->nWifim-4);
    float delay = history_length*envStepTime;
    if (this->nWifim > 5)
    {
        // Force mixed scenario
        //udp = true;         // UDP
        //payloadSize = 1472; // bytes
        //direction = 1;      // DL
        for (int i = 0; i < 5; ++i)
        {
            if (udp)
            {
                switch(direction)
                {
                    case 0: // UL
                        installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, this->offeredLoad, 0.0 , simulationTime
                            + 2 + delay);
                        break;
                    case 1: // DL
                        installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, this->offeredLoad, 0.0 , simulationTime
                            + 2 + delay);
                        break;
                    case 2: // UL+DL
                        installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, this->offeredLoad, 0.0 , simulationTime
                            + 2 + delay);
                        installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, this->offeredLoad, 0.0 , simulationTime
                            + 2 + delay);
                        break;
                }
            }
            else // TCP
            {
                switch(direction)
                {
                    case 0: // UL
                        installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                            simulationTime + 2 + delay);
                        break;
                    case 1: // DL
                        installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize, 0.0,
                            simulationTime + 2 + delay);
                        break;
                    case 2: // UL+DL
                        installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize, 0.0,
                            simulationTime + 2 + delay);
                        installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize, 0.0,
                            simulationTime + 2 + delay);
                        break;
                }
            }
        }
        // Force mixed scenario
        //udp = false;        // TCP
        //payloadSize = 1460; // bytes
        //direction = 2;      // UL+DL
        for (int i = 5; i < this->nWifim; ++i)
        {
            if (udp)
            {
                switch(direction)
                {
                    case 0: // UL
                        installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, this->offeredLoad, delay + (i - 4) * delta,
                            simulationTime + 2 + delay);
                        break;
                    case 1: // DL
                        installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, this->offeredLoad, delay + (i - 4) * delta,
                            simulationTime + 2 + delay);
                        break;
                    case 2: // UL+DL
                        installUDPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, payloadSize, this->offeredLoad, delay + (i - 4) * delta,
                            simulationTime + 2 + delay);
                        installUDPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->port++, payloadSize, this->offeredLoad, delay + (i - 4) * delta,
                            simulationTime + 2 + delay);
                        break;
                }
            }
            else // TCP
            {
                switch(direction)
                {
                    case 0: // UL
                        installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize,
                            delay + (i - 4) * delta, simulationTime + 2 + delay);
                        break;
                    case 1: // DL
                        installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize,
                            delay + (i - 4) * delta, simulationTime + 2 + delay);
                        break;
                    case 2: // UL+DL
                        installTCPTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->apNodeInterface.GetAddress(0), this->port++, payloadSize,
                            delay + (i - 4) * delta, simulationTime + 2 + delay);
                        installTCPTrafficGenerator(this->wifiApNode.Get(0), this->wifiStaNode.Get(i), this->staNodeInterface.GetAddress(i), this->port++, payloadSize,
                            delay + (i - 4) * delta, simulationTime + 2 + delay);
                        break;
                }
            }
        }
    }
    else
    {
        std::cout << "Not enough Wi-Fi stations to support the convergence scenario." << endl;
        exit(0);
    }
}
#endif
