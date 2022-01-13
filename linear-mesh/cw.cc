#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/opengym-module.h"
//#include "ns3/csma-module.h"
#include "ns3/propagation-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/yans-wifi-channel.h"

#include <fstream>
#include <string>
#include <math.h>
#include <ctime>   //timestampi
#include <iomanip> // put_time
#include <deque>
#include <algorithm>
#include <csignal>
#include "scenario.h"

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OpenGym");

void PopulateARPcache();
void recordHistory();

double envStepTime = 0.1;
double simulationTime = 10; //seconds
double current_time = 0.0;
bool verbose = false;
int end_delay = 0;
bool dry_run = false;

Ptr<FlowMonitor> monitor;
FlowMonitorHelper flowmon;

uint32_t CW = 0; // CW for legacy devices
uint32_t CW_ax = 0; // CW for 802.11ax devices
uint32_t historyLength = 20;
string type = "discrete";
bool non_zero_start = false;
Scenario *wifiScenario;

deque<float> history;

bool uplink;
bool udp;
int payloadSize;
uint64_t g_rxPktNum = 0;
uint64_t g_txPktNum = 0;
double len = 9.8; //meters
double wid = 7.4; //meters

Ptr<ListPositionAllocator> getCoordinates (uint16_t numStas) {
        double side = sqrt (len * wid / numStas); // Side of the square of each STA
        uint16_t stasInLen = round (len / side); // Number of STAs in length
        uint16_t stasInWid = round (wid / side); // Number of STAs in width
        if (stasInLen * stasInWid < numStas) {
                stasInWid ++;
        }
        double sepInLen = len / stasInLen;
        double sepInWid = wid / stasInWid;
        double i = sepInLen / 2;
        double j = sepInWid / 2;
        uint16_t counter = 1;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
        positionAlloc->Add (Vector (len / 2, wid / 2, 2)); // AP coordinates
        while (i < len) {
                while (j < wid) {
                        if (counter <= numStas) {
                                positionAlloc->Add (Vector (i, j, 0)); // STA k coordinates
                        }
                        j += sepInWid;
                        counter++;
                }
                i += sepInLen;
                j = sepInWid / 2;
        }
        return positionAlloc;
}

/*
Define observation space
*/
Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
    current_time += envStepTime;

    float low = 0.0;
    float high = 10.0;
    std::vector<uint32_t> shape = {
        historyLength,
    };
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    if (verbose)
        NS_LOG_UNCOND("MyGetObservationSpace: " << space);
    return space;
}

/*
Define action space
*/
Ptr<OpenGymSpace> MyGetActionSpace(void)
{
    float low = 0.0;
    float high = 10.0;
    std::vector<uint32_t> shape = {
        2,
    };
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    if (verbose)
        NS_LOG_UNCOND("MyGetActionSpace: " << space);
    return space;
}

/*
Define extra info. Optional
*/
double jain_index(Address apAddress)
{
    double flowThr;
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    double nominator;
    double denominator;
    double n=0;
    double station_id = 0;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {

        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        Address srcAddress = t.sourceAddress;
        bool cond = uplink ? srcAddress != apAddress : srcAddress == apAddress; // Do not process TCP ACK flows

        if (cond)
        {
            flowThr = i->second.rxBytes;
            flowThr /= wifiScenario->getStationUptime(station_id, current_time);
            if(flowThr > 0)
            {
                nominator += flowThr;
                denominator += flowThr*flowThr;
                n++;
            }
            station_id++;
        }
    }
    nominator *= nominator;
    denominator *= n;
    return nominator/denominator;
}

std::string MyGetExtraInfo(Address addr)
{
    // A static variable declared within a function scope will be created and initialized once for all non-recursive calls to that function.
    // https://stackoverflow.com/questions/35926569/static-float-variable-only-stores-highest-value-assigned-need-it-to-store-most
    static float ticks = 0.0;
    static float lastValue = 0.0;
    float obs = g_rxPktNum - lastValue;
    lastValue = g_rxPktNum;
    ticks += envStepTime;

    float sentMbytes = obs * payloadSize * 8.0 / 1024 / 1024;

    std::string myInfo = std::to_string(sentMbytes);
    myInfo = myInfo + "|" + to_string(CW) + "|";
    myInfo = myInfo + to_string(CW_ax) + "|";
    myInfo = myInfo + to_string(wifiScenario->getActiveStationCount(ticks)) + "|";
    myInfo = myInfo + to_string(jain_index(addr));

    if (verbose)
        NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);

    string filename("cw-values.csv");
    fstream file;

    file.open(filename, std::ios_base::app | std::ios_base::in);
    if (file.is_open())
        file << CW << "," << CW_ax << endl;

    return myInfo;
}

/*
Execute received actions
*/
bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    if (verbose)
        NS_LOG_UNCOND("MyExecuteActions: " << action);

    Ptr<OpenGymBoxContainer<float>> box = DynamicCast<OpenGymBoxContainer<float>>(action);
    std::vector<float> actionVector = box->GetData();

    // TODO: - 1 after pow not implemented (see paper)
    if (type == "discrete")
    {
        CW = pow(2, actionVector.at(0) + 4);
        CW_ax = pow(2, actionVector.at(1) + 4);
    }
    else if (type == "continuous")
    {
        CW = pow(2, actionVector.at(0) + 4); // TODO: Floor not implemented (see paper)
        CW_ax = pow(2, actionVector.at(1) + 4);
    }
    else if (type == "direct_continuous")
    {
        CW = actionVector.at(0);
        CW_ax = actionVector.at(1);
    }
    else
    {
        std::cout << "Unsupported agent type!" << endl;
        exit(0);
    }

    uint32_t min_cw = 16;
    uint32_t max_cw = 1024;

    CW = min(max_cw, max(CW, min_cw));
    CW_ax = min(max_cw, max(CW_ax, min_cw));

    if(!dry_run){
        // Set CW for legacy devices
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW));
        // Set CW for 802.11ax devices (STA 0)
        Config::Set("/$ns3::NodeListPriv/NodeList/0/$ns3::Node/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW_ax));
        Config::Set("/$ns3::NodeListPriv/NodeList/0/$ns3::Node/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW_ax));
    }
    return true;
}

float MyGetReward(void)
{
    static float ticks = 0.0;
    static uint32_t last_packets = 0;
    static float last_reward = 0.0;
    ticks += envStepTime;

    float res = g_rxPktNum - last_packets;
    float reward = res * payloadSize * 8.0 / 1024 / 1024 / (5 * 150 * envStepTime) * 10;
    //NS_LOG_UNCOND("\n### thr: " << res * payloadSize * 8.0 / 1024 / 1024 / envStepTime);
    //NS_LOG_UNCOND("### thr_max: " << 5 * 150 / 10);
    //NS_LOG_UNCOND("### reward: " << reward);

    last_packets = g_rxPktNum;

    if (ticks <= 2 * envStepTime)
        return 0.0;

    if (verbose)
        NS_LOG_UNCOND("MyGetReward: " << reward);

    if(reward>1.0f || reward<0.0f)
        reward = last_reward;
    last_reward = reward;
    return last_reward;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer> MyGetObservation()
{
    recordHistory();

    std::vector<uint32_t> shape = {
        historyLength,
    };
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);

    for (uint32_t i = 0; i < history.size(); i++)
    {
        if (history[i] >= -100 && history[i] <= 100)
            box->AddValue(history[i]);
        else
            box->AddValue(0);
    }
    for (uint32_t i = history.size(); i < historyLength; i++)
    {
        box->AddValue(0);
    }
    if (verbose)
        NS_LOG_UNCOND("MyGetObservation: " << box);
    return box;
}

bool MyGetGameOver(void)
{
    // bool isGameOver = (ns3::Simulator::Now().GetSeconds() > simulationTime + end_delay + 1.0);
    return false;
}

void ScheduleNextStateRead(int nWifi, double envStepTime, Ptr<OpenGymInterface> openGymInterface)
{
    int totalRx = 0;
    for (int i=0 ; i < nWifi ; ++i)
    {
        if (udp)
        {
            int rx = DynamicCast<UdpServer> (serversApp.Get (i))->GetReceived (); // Number of received packets
            if (verbose)
            {
                NS_LOG_UNCOND("Packets of STA " << i << ": " << rx);
            }
            totalRx += rx;
        }
        else
        {
            int rx = static_cast<int> (DynamicCast<PacketSink> (serversApp.Get (i))->GetTotalRx () / payloadSize); // Number of received packets
            if (verbose)
            {
                NS_LOG_UNCOND("Packets of STA " << i << ": " << rx);
            }
            totalRx += rx;
        }
    }
    g_rxPktNum = totalRx; // Update received packets
    if (verbose)
    {
        NS_LOG_UNCOND("\nTotal transmitted packets:\t" << g_txPktNum);
        NS_LOG_UNCOND("Total received packets:\t\t" << g_rxPktNum);
    }

    Simulator::Schedule(Seconds(envStepTime), &ScheduleNextStateRead, nWifi, envStepTime, openGymInterface);
    openGymInterface->NotifyCurrentState();
}

void recordHistory()
{
    static uint32_t last_rx = 0;
    static uint32_t last_tx = 0;
    static uint32_t calls = 0;
    calls++;

    float received = g_rxPktNum - last_rx;
    float sent = g_txPktNum - last_tx;
    float errs = sent - received;
    float ratio;

    ratio = errs / sent;
    history.push_front(ratio);

    if (history.size() > historyLength)
    {
        history.pop_back();
    }
    last_rx = g_rxPktNum;
    last_tx = g_txPktNum;

    if (calls < historyLength && non_zero_start)
    {
        Simulator::Schedule(Seconds(envStepTime), &recordHistory);
    }
    else if (calls == historyLength && non_zero_start)
    {
        g_rxPktNum = 0;
        g_txPktNum = 0;
        last_rx = 0;
        last_tx = 0;
    }
}

void packetSent(Ptr<const Packet> packet)
{
    g_txPktNum++;
}

void set_phy(int nWifi, int nAx, int guardInterval, NodeContainer &staNodes, NodeContainer &apNode, YansWifiPhyHelper &phy)
{
    Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel>();
    lossModel->SetDefaultLoss(50);

    staNodes.Create(nWifi - nAx + 1);
    apNode.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    Ptr<YansWifiChannel> chan = channel.Create();
    chan->SetPropagationLossModel(lossModel);
    chan->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    phy = YansWifiPhyHelper::Default();
    phy.SetChannel(chan);

    // Set guard interval
    phy.Set("GuardInterval", TimeValue(NanoSeconds(guardInterval)));
}

void set_nodes(int nWifi, int nAx, int channelWidth, int rng, int32_t simSeed, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer &stasInterface,
    Ipv4InterfaceContainer &apInterface, YansWifiPhyHelper phy, WifiMacHelper mac, WifiHelper wifi, NetDeviceContainer &apDevice, NetDeviceContainer &staDevice)
{
    Ssid ssid = Ssid("ns3-80211ax");

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false),
                "BE_MaxAmpduSize", UintegerValue(0)); // Disable A-MPDU aggregation for STAs

    staDevice = wifi.Install(phy, mac, staNodes);

    mac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(ssid),
                "BE_MaxAmpduSize", UintegerValue(65535)); // Enable A-MPDU aggregation for AP (https://www.nsnam.org/doxygen/classns3_1_1_ap_wifi_mac.html)

    apDevice = wifi.Install(phy, mac, apNode);

    // Set channel width
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(channelWidth));

    // Mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator (getCoordinates(nWifi - nAx + 1));
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (apNode);
    mobility.Install (staNodes);

    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install(apNode);
    stack.Install(staNodes);

    //Random
    if(simSeed!=-1)
        RngSeedManager::SetSeed(simSeed);
    RngSeedManager::SetRun(rng);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    stasInterface = address.Assign(staDevice);
    apInterface = address.Assign(apDevice);

    if (!dry_run)
    {
        // Set CW for legacy devices
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW));
        // Set CW for 802.11ax devices (STA 0)
        Config::Set("/$ns3::NodeListPriv/NodeList/0/$ns3::Node/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW_ax));
        Config::Set("/$ns3::NodeListPriv/NodeList/0/$ns3::Node/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW_ax));
    }
    else
    {
        NS_LOG_UNCOND("Default CW");
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(16));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(1024));
    }
}

void set_sim(bool tracing, bool dry_run, int warmup, uint32_t openGymPort, YansWifiPhyHelper phy, NetDeviceContainer apDevice, NetDeviceContainer staDevice,
    int end_delay, Ptr<FlowMonitor> &monitor, FlowMonitorHelper &flowmon, int nWifi, Address addr)
{
    monitor = flowmon.InstallAll();
    monitor->SetAttribute("StartTime", TimeValue(Seconds(warmup)));

    if (tracing)
    {
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.EnablePcap("cw-" + std::to_string(std::time(0)), apDevice.Get(0));
    }

    Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface>(openGymPort);
    openGymInterface->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
    openGymInterface->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
    openGymInterface->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
    openGymInterface->SetGetObservationCb(MakeCallback(&MyGetObservation));
    openGymInterface->SetGetRewardCb(MakeCallback(&MyGetReward));
    openGymInterface->SetGetExtraInfoCb(MakeBoundCallback(&MyGetExtraInfo, addr));
    openGymInterface->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));

    if (non_zero_start)
    {
        Simulator::Schedule(Seconds(1.0), &recordHistory);
        Simulator::Schedule(Seconds(envStepTime * historyLength + 1.0), &ScheduleNextStateRead, nWifi, envStepTime, openGymInterface);
    }
    else
        Simulator::Schedule(Seconds(1.0), &ScheduleNextStateRead, nWifi, envStepTime, openGymInterface);

    Simulator::Stop(Seconds(simulationTime + end_delay + 1.0 + envStepTime*(historyLength+1)));

    NS_LOG_UNCOND("Simulation started");
    Simulator::Run();
}

void signalHandler(int signum)
{
    cout << "Interrupt signal " << signum << " received.\n";
    exit(signum);
}

int main(int argc, char *argv[])
{
    int nWifi = 5;
    int nAx = 2;
    bool tracing = true;
    bool useRts = false;
    //int mcs = 11;
    int channelWidth = 20;
    int guardInterval = 800;
    string dataRate = "150";
    int port = 1025;
    uplink = true;
    udp = true;
    string outputCsv = "cw.csv";
    string scenario = "basic";

    int rng = 1;
    int warmup = 1;

    uint32_t openGymPort = 5555;
    int32_t simSeed = -1;

    signal(SIGTERM, signalHandler);

    CommandLine cmd;
    cmd.AddValue("openGymPort", "Specify port number. Default: 5555", openGymPort);
    cmd.AddValue("CW", "Value of Contention Window for legacy devices", CW);
    cmd.AddValue("CW_ax", "Value of Contention Window for 802.11ax devices", CW_ax);
    cmd.AddValue("historyLength", "Length of history window", historyLength);
    cmd.AddValue("nWifi", "Number of wifi 802.11ac STA devices", nWifi);
    cmd.AddValue("nAx", "Number of OFDMA 802.11ax STA devices", nAx);
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("rng", "Number of RngRun", rng);
    cmd.AddValue("uplink", "Uplink direction if 'true', downlink if 'false'", uplink);
    cmd.AddValue("udp", "UDP traffic if 'true', TCP if 'false'", udp);
    cmd.AddValue("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
    cmd.AddValue("envStepTime", "Step time in seconds. Default: 0.1s", envStepTime);
    cmd.AddValue("agentType", "Type of agent actions: discrete, continuous", type);
    cmd.AddValue("nonZeroStart", "Start only after history buffer is filled", non_zero_start);
    cmd.AddValue("scenario", "Scenario for analysis: basic, convergence, reaction", scenario);
    cmd.AddValue("dryRun", "Execute scenario with BEB and no agent interaction", dry_run);
    cmd.AddValue("seed", "Random seed", simSeed);

    cmd.Parse(argc, argv);

    if (nAx > nWifi)
    {
        NS_LOG_UNCOND("ERROR! 'nAx' has to be less or equal to 'nWifi'");
        exit(1);
    }

    // Payload size = MTU - IPv4 header - (TCP or UDP) header
    // Headers' values taken from: https://cs.fit.edu/~mmahoney/cse4232/tcpip.html
    payloadSize = udp ? 1472 : 1460; // bytes

    NS_LOG_UNCOND("Ns3Env parameters:");
    NS_LOG_UNCOND("--nWifi: " << nWifi);
    NS_LOG_UNCOND("--nAx: " << nAx);
    NS_LOG_UNCOND("--simulationTime: " << simulationTime);
    NS_LOG_UNCOND("--openGymPort: " << openGymPort);
    NS_LOG_UNCOND("--envStepTime: " << envStepTime);
    NS_LOG_UNCOND("--uplink: " << uplink);
    NS_LOG_UNCOND("--udp: " << udp);
    NS_LOG_UNCOND("--seed: " << simSeed);
    NS_LOG_UNCOND("--agentType: " << type);
    NS_LOG_UNCOND("--scenario: " << scenario);
    NS_LOG_UNCOND("--dryRun: " << dry_run);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
    }

    // Disable fragmentation
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2346")); // bytes (max 802.11 MAC frame)

    NodeContainer staNodes;
    NodeContainer apNode;
    Ipv4InterfaceContainer stasInterface;
    Ipv4InterfaceContainer apInterface;
    YansWifiPhyHelper phy;
    set_phy(nWifi, nAx, guardInterval, staNodes, apNode, phy);

    WifiMacHelper mac;
    WifiHelper wifi;

    //802.11ax PHY
    //wifi.SetStandard(WIFI_PHY_STANDARD_80211ax_5GHZ);
    //std::ostringstream oss;
    //oss << "HeMcs" << mcs;
    //wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
    //                             "ControlMode", StringValue(oss.str()));

    //802.11ac PHY
    phy.Set ("ShortGuardEnabled", BooleanValue (0));
    wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue ("VhtMcs8"),
                                  "ControlMode", StringValue ("VhtMcs8"));

    //802.11n PHY
    //phy.Set ("ShortGuardEnabled", BooleanValue (1));
    //wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
    //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    //                              "DataMode", StringValue ("HtMcs7"),
    //                              "ControlMode", StringValue ("HtMcs7"));

    NetDeviceContainer apDevice;
    NetDeviceContainer staDevice;
    set_nodes(nWifi, nAx, channelWidth, rng, simSeed, staNodes, apNode, stasInterface, apInterface, phy, mac, wifi, apDevice, staDevice);

    ScenarioFactory helper = ScenarioFactory(nWifi, nAx, staNodes, apNode, stasInterface, apInterface, port, dataRate, historyLength);
    wifiScenario = helper.getScenario(scenario);

    // if (!dry_run)
    // {
    if (non_zero_start)
        end_delay = envStepTime * historyLength + 1.0;
    else
        end_delay = 0.0;
    // }

    wifiScenario->installScenario(simulationTime + end_delay + envStepTime, envStepTime, uplink, udp, payloadSize);

    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx", MakeCallback(&packetSent));
    // Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&packetSent));

    wifiScenario->PopulateARPcache();
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    set_sim(tracing, dry_run, warmup, openGymPort, phy, apDevice, staDevice, end_delay, monitor, flowmon, nWifi, apInterface.GetAddress(0));

    double flowThr;
    float res =  g_rxPktNum * payloadSize * 8.0 / 1024 / 1024;
    printf("Sent mbytes: %.2f\tThroughput: %.3f", res, res/simulationTime);
    ofstream myfile;
    myfile.open(outputCsv, ios::app);

    /* Contents of CSV output file
    Timestamp, CW, CW_ax, nWifi, RngRun, SourceIP, DestinationIP, Throughput
    */
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        auto time = std::time(nullptr); //Get timestamp
        auto tm = *std::localtime(&time);
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        flowThr = i->second.rxBytes * 8.0 / simulationTime / 1000 / 1000;
        NS_LOG_UNCOND("Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\tThroughput: " << flowThr << " Mbps\tTime: " << i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds() << " s\tRx packets " << i->second.rxPackets);
        myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << CW << "," << CW_ax << "," << nWifi << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << t.destinationAddress << "," << flowThr;
        myfile << std::endl;
    }
    myfile.close();

    Simulator::Destroy();
    NS_LOG_UNCOND("Packets registered by handler: " << g_rxPktNum << " Packets" << endl);

    return 0;
}
