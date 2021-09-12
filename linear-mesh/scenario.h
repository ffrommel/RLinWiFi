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

string filename[3] = {"tx_pkts.csv", "rx_pkts.csv", "cw_values.csv"};
ofstream outputFile[3];

uint64_t g_rxPktNum = 0;
vector<int> rxPkts(50, 0);
vector<int> effective_stas(50, 0);

// --- START PACKET RECEIVED FUNCIONS ---
void packetReceived0(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[0]++;
    effective_stas[0]++;

    if (outputFile[1].is_open())
        outputFile[1] << "0," << rxPkts[0] << endl;
}

void packetReceived1(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[1]++;
    effective_stas[1]++;

    if (outputFile[1].is_open())
        outputFile[1] << "1," << rxPkts[1] << endl;
}

void packetReceived2(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[2]++;
    effective_stas[2]++;

    if (outputFile[1].is_open())
        outputFile[1] << "2," << rxPkts[2] << endl;
}

void packetReceived3(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[3]++;
    effective_stas[3]++;

    if (outputFile[1].is_open())
        outputFile[1] << "3," << rxPkts[3] << endl;
}

void packetReceived4(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[4]++;
    effective_stas[4]++;

    if (outputFile[1].is_open())
        outputFile[1] << "4," << rxPkts[4] << endl;
}

void packetReceived5(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[5]++;
    effective_stas[5]++;

    if (outputFile[1].is_open())
        outputFile[1] << "5," << rxPkts[5] << endl;
}

void packetReceived6(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[6]++;
    effective_stas[6]++;

    if (outputFile[1].is_open())
        outputFile[1] << "6," << rxPkts[6] << endl;
}

void packetReceived7(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[7]++;
    effective_stas[7]++;

    if (outputFile[1].is_open())
        outputFile[1] << "7," << rxPkts[7] << endl;
}

void packetReceived8(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[8]++;
    effective_stas[8]++;

    if (outputFile[1].is_open())
        outputFile[1] << "8," << rxPkts[8] << endl;
}

void packetReceived9(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[9]++;
    effective_stas[9]++;

    if (outputFile[1].is_open())
        outputFile[1] << "9," << rxPkts[9] << endl;
}

void packetReceived10(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[10]++;
    effective_stas[10]++;

    if (outputFile[1].is_open())
        outputFile[1] << "10," << rxPkts[10] << endl;
}

void packetReceived11(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[11]++;
    effective_stas[11]++;

    if (outputFile[1].is_open())
        outputFile[1] << "11," << rxPkts[11] << endl;
}

void packetReceived12(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[12]++;
    effective_stas[12]++;

    if (outputFile[1].is_open())
        outputFile[1] << "12," << rxPkts[12] << endl;
}

void packetReceived13(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[13]++;
    effective_stas[13]++;

    if (outputFile[1].is_open())
        outputFile[1] << "13," << rxPkts[13] << endl;
}

void packetReceived14(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[14]++;
    effective_stas[14]++;

    if (outputFile[1].is_open())
        outputFile[1] << "14," << rxPkts[14] << endl;
}

void packetReceived15(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[15]++;
    effective_stas[15]++;

    if (outputFile[1].is_open())
        outputFile[1] << "15," << rxPkts[15] << endl;
}

void packetReceived16(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[16]++;
    effective_stas[16]++;

    if (outputFile[1].is_open())
        outputFile[1] << "16," << rxPkts[16] << endl;
}

void packetReceived17(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[17]++;
    effective_stas[17]++;

    if (outputFile[1].is_open())
        outputFile[1] << "17," << rxPkts[17] << endl;
}

void packetReceived18(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[18]++;
    effective_stas[18]++;

    if (outputFile[1].is_open())
        outputFile[1] << "18," << rxPkts[18] << endl;
}

void packetReceived19(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[19]++;
    effective_stas[19]++;

    if (outputFile[1].is_open())
        outputFile[1] << "19," << rxPkts[19] << endl;
}

void packetReceived20(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[20]++;
    effective_stas[20]++;

    if (outputFile[1].is_open())
        outputFile[1] << "20," << rxPkts[20] << endl;
}

void packetReceived21(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[21]++;
    effective_stas[21]++;

    if (outputFile[1].is_open())
        outputFile[1] << "21," << rxPkts[21] << endl;
}

void packetReceived22(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[22]++;
    effective_stas[22]++;

    if (outputFile[1].is_open())
        outputFile[1] << "22," << rxPkts[22] << endl;
}

void packetReceived23(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[23]++;
    effective_stas[23]++;

    if (outputFile[1].is_open())
        outputFile[1] << "23," << rxPkts[23] << endl;
}

void packetReceived24(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[24]++;
    effective_stas[24]++;

    if (outputFile[1].is_open())
        outputFile[1] << "24," << rxPkts[24] << endl;
}

void packetReceived25(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[25]++;
    effective_stas[25]++;

    if (outputFile[1].is_open())
        outputFile[1] << "25," << rxPkts[25] << endl;
}

void packetReceived26(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[26]++;
    effective_stas[26]++;

    if (outputFile[1].is_open())
        outputFile[1] << "26," << rxPkts[26] << endl;
}

void packetReceived27(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[27]++;
    effective_stas[27]++;

    if (outputFile[1].is_open())
        outputFile[1] << "27," << rxPkts[27] << endl;
}

void packetReceived28(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[28]++;
    effective_stas[28]++;

    if (outputFile[1].is_open())
        outputFile[1] << "28," << rxPkts[28] << endl;
}

void packetReceived29(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[29]++;
    effective_stas[29]++;

    if (outputFile[1].is_open())
        outputFile[1] << "29," << rxPkts[29] << endl;
}

void packetReceived30(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[30]++;
    effective_stas[30]++;

    if (outputFile[1].is_open())
        outputFile[1] << "30," << rxPkts[30] << endl;
}

void packetReceived31(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[31]++;
    effective_stas[31]++;

    if (outputFile[1].is_open())
        outputFile[1] << "31," << rxPkts[31] << endl;
}

void packetReceived32(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[32]++;
    effective_stas[32]++;

    if (outputFile[1].is_open())
        outputFile[1] << "32," << rxPkts[32] << endl;
}

void packetReceived33(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[33]++;
    effective_stas[33]++;

    if (outputFile[1].is_open())
        outputFile[1] << "33," << rxPkts[33] << endl;
}

void packetReceived34(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[34]++;
    effective_stas[34]++;

    if (outputFile[1].is_open())
        outputFile[1] << "34," << rxPkts[34] << endl;
}

void packetReceived35(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[35]++;
    effective_stas[35]++;

    if (outputFile[1].is_open())
        outputFile[1] << "35," << rxPkts[35] << endl;
}

void packetReceived36(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[36]++;
    effective_stas[36]++;

    if (outputFile[1].is_open())
        outputFile[1] << "36," << rxPkts[36] << endl;
}

void packetReceived37(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[37]++;
    effective_stas[37]++;

    if (outputFile[1].is_open())
        outputFile[1] << "37," << rxPkts[37] << endl;
}

void packetReceived38(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[38]++;
    effective_stas[38]++;

    if (outputFile[1].is_open())
        outputFile[1] << "38," << rxPkts[38] << endl;
}

void packetReceived39(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[39]++;
    effective_stas[39]++;

    if (outputFile[1].is_open())
        outputFile[1] << "39," << rxPkts[39] << endl;
}

void packetReceived40(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[40]++;
    effective_stas[40]++;

    if (outputFile[1].is_open())
        outputFile[1] << "40," << rxPkts[40] << endl;
}

void packetReceived41(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[41]++;
    effective_stas[41]++;

    if (outputFile[1].is_open())
        outputFile[1] << "41," << rxPkts[41] << endl;
}

void packetReceived42(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[42]++;
    effective_stas[42]++;

    if (outputFile[1].is_open())
        outputFile[1] << "42," << rxPkts[42] << endl;
}

void packetReceived43(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[43]++;
    effective_stas[43]++;

    if (outputFile[1].is_open())
        outputFile[1] << "43," << rxPkts[43] << endl;
}

void packetReceived44(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[44]++;
    effective_stas[44]++;

    if (outputFile[1].is_open())
        outputFile[1] << "44," << rxPkts[44] << endl;
}

void packetReceived45(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[45]++;
    effective_stas[45]++;

    if (outputFile[1].is_open())
        outputFile[1] << "45," << rxPkts[45] << endl;
}

void packetReceived46(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[46]++;
    effective_stas[46]++;

    if (outputFile[1].is_open())
        outputFile[1] << "46," << rxPkts[46] << endl;
}

void packetReceived47(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[47]++;
    effective_stas[47]++;

    if (outputFile[1].is_open())
        outputFile[1] << "47," << rxPkts[47] << endl;
}

void packetReceived48(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[48]++;
    effective_stas[48]++;

    if (outputFile[1].is_open())
        outputFile[1] << "48," << rxPkts[48] << endl;
}

void packetReceived49(Ptr<const Packet> packet)
{
    g_rxPktNum++;
    rxPkts[49]++;
    effective_stas[49]++;

    if (outputFile[1].is_open())
        outputFile[1] << "49," << rxPkts[49] << endl;
}
// --- END PACKET RECEIVED FUNCIONS ---

class Scenario
{
  protected:
    int nWifim;
    NodeContainer wifiStaNode;
    NodeContainer wifiApNode;
    int port;
    std::string offeredLoad;
    std::vector<double> start_times;
    std::vector<double> end_times;
    int history_length;

    void installTrafficGenerator(ns3::Ptr<ns3::Node> fromNode,
                                 ns3::Ptr<ns3::Node> toNode,
                                 int port,
                                 std::string offeredLoad,
                                 double startTime,
                                 double endTime);

  public:
    Scenario(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, int port, std::string offeredLoad, int history_length);
    virtual void installScenario(double simulationTime, double envStepTime) = 0;
    void PopulateARPcache();
    int getActiveStationCount(double time);
    float getStationUptime(int id, double time);
};

class BasicScenario : public Scenario
{
    using Scenario::Scenario;

  public:
    void installScenario(double simulationTime, double envStepTime) override;
};

class ConvergenceScenario : public Scenario
{
    using Scenario::Scenario;

  public:
    void installScenario(double simulationTime, double envStepTime) override;
};

class ScenarioFactory
{
  private:
    int nWifim;
    NodeContainer wifiStaNode;
    NodeContainer wifiApNode;
    int port;
    int history_length;
    std::string offeredLoad;

  public:
    ScenarioFactory(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, int port, std::string offeredLoad, int history_length)
    {
        this->nWifim = nWifim;
        this->wifiStaNode = wifiStaNode;
        this->wifiApNode = wifiApNode;
        this->port = port;
        this->offeredLoad = offeredLoad;
        this->history_length = history_length;
    }

    Scenario *getScenario(std::string scenario)
    {
        Scenario *wifiScenario;
        if (scenario == "basic")
        {
            wifiScenario = new BasicScenario(this->nWifim, this->wifiStaNode, this->wifiApNode, this->port, this->offeredLoad, this->history_length);
        }
        else if (scenario == "convergence")
        {
            wifiScenario = new ConvergenceScenario(this->nWifim, this->wifiStaNode, this->wifiApNode, this->port, this->offeredLoad, this->history_length);
        }
        else
        {
            std::cout << "Unsupported scenario" << endl;
            exit(0);
        }
        return wifiScenario;
    }
};

Scenario::Scenario(int nWifim, NodeContainer wifiStaNode, NodeContainer wifiApNode, int port, std::string offeredLoad, int history_length)
{
    this->nWifim = nWifim;
    this->wifiStaNode = wifiStaNode;
    this->wifiApNode = wifiApNode;
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
    // int res=0;
    // for(uint i=0; i<start_times.size(); i++)
    //     if(start_times.at(i)<time && time<end_times.at(i))
    //         res++;
    // return res;
}

void Scenario::installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, string offeredLoad, double startTime, double endTime)
{
    start_times.push_back(startTime);
    end_times.push_back(endTime);

    Ptr<Ipv4> ipv4 = toNode->GetObject<Ipv4>();           // Get Ipv4 instance of the node
    Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.

    ApplicationContainer sourceApplications, sinkApplications;

    uint8_t tosValue = 0x70; //AC_BE
    //Add random fuzz to app start time
    double min = 0.0;
    double max = 1.0;
    Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable>();
    fuzz->SetAttribute("Min", DoubleValue(min));
    fuzz->SetAttribute("Max", DoubleValue(max));

    InetSocketAddress sinkSocket(addr, port);
    sinkSocket.SetTos(tosValue);
    //OnOffHelper onOffHelper ("ns3::TcpSocketFactory", sinkSocket);
    OnOffHelper onOffHelper("ns3::UdpSocketFactory", sinkSocket);
    onOffHelper.SetConstantRate(DataRate(offeredLoad + "Mbps"), 1500 - 20 - 8 - 8);
    // onOffHelper.TraceConnectWithoutContext("Tx", MakeCallback(&packetSent));
    sourceApplications.Add(onOffHelper.Install(fromNode)); //fromNode

    //PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", sinkSocket);
    // PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
    UdpServerHelper sink(port);
    sinkApplications = sink.Install(toNode);
    // sinkApplications.Add (packetSinkHelper.Install (toNode)); //toNode

    sinkApplications.Start(Seconds(startTime));
    sinkApplications.Stop(Seconds(endTime));

    Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(sinkApplications.Get(0));

    switch(fromNode->GetId()) {
        case 0:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived0));
            break;
        case 1:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived1));
            break;
        case 2:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived2));
            break;
        case 3:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived3));
            break;
        case 4:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived4));
            break;
        case 5:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived5));
            break;
        case 6:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived6));
            break;
        case 7:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived7));
            break;
        case 8:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived8));
            break;
        case 9:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived9));
            break;
        case 10:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived10));
            break;
        case 11:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived11));
            break;
        case 12:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived12));
            break;
        case 13:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived13));
            break;
        case 14:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived14));
            break;
        case 15:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived15));
            break;
        case 16:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived16));
            break;
        case 17:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived17));
            break;
        case 18:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived18));
            break;
        case 19:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived19));
            break;
        case 20:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived20));
            break;
        case 21:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived21));
            break;
        case 22:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived22));
            break;
        case 23:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived23));
            break;
        case 24:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived24));
            break;
        case 25:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived25));
            break;
        case 26:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived26));
            break;
        case 27:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived27));
            break;
        case 28:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived28));
            break;
        case 29:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived29));
            break;
        case 30:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived30));
            break;
        case 31:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived31));
            break;
        case 32:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived32));
            break;
        case 33:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived33));
            break;
        case 34:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived34));
            break;
        case 35:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived35));
            break;
        case 36:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived36));
            break;
        case 37:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived37));
            break;
        case 38:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived38));
            break;
        case 39:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived39));
            break;
        case 40:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived40));
            break;
        case 41:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived41));
            break;
        case 42:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived42));
            break;
        case 43:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived43));
            break;
        case 44:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived44));
            break;
        case 45:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived45));
            break;
        case 46:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived46));
            break;
        case 47:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived47));
            break;
        case 48:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived48));
            break;
        case 49:
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&packetReceived49));
            break;
    }

    sourceApplications.Start(Seconds(startTime));
    sourceApplications.Stop(Seconds(endTime));
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

void BasicScenario::installScenario(double simulationTime, double envStepTime)
{
    for (int i = 0; i < this->nWifim; ++i)
    {
        installTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, this->offeredLoad, 0.0, simulationTime + 2 + envStepTime*history_length);
    }
}

void ConvergenceScenario::installScenario(double simulationTime, double envStepTime)
{
    float delta = simulationTime/(this->nWifim-4);
    float delay = history_length*envStepTime;
    if (this->nWifim > 5)
    {
        for (int i = 0; i < 5; ++i)
        {
            installTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, this->offeredLoad, 0.0 , simulationTime + 2 + delay);
        }
        for (int i = 5; i < this->nWifim; ++i)
        {
            installTrafficGenerator(this->wifiStaNode.Get(i), this->wifiApNode.Get(0), this->port++, this->offeredLoad, delay+(i - 4) * delta, simulationTime + 2 + delay);
        }
    }
    else
    {
        std::cout << "Not enough Wi-Fi stations to support the convergence scenario." << endl;
        exit(0);
    }
}
#endif
