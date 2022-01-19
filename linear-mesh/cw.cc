#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/opengym-module.h"
#include "ns3/propagation-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/yans-wifi-channel.h"

#include <fstream>
#include <string>
#include <math.h>
#include <ctime>
#include <iomanip>
#include <deque>
#include <algorithm>
#include <csignal>
#include "scenario.h"

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OpenGym");

void PopulateARPcache();
void recordHistory();

bool new_state = false;         // true: state = p_col + n, false: state = p_col
bool dry_run = false;
uint32_t CW = 0;		// CW for legacy devices
uint32_t CW_ax = 0;		// CW for 802.11ax devices
double simulationTime = 10;     // seconds
double envStepTime = 0.1;       // seconds
double current_time = 0.0;      // seconds
uint32_t history_length = 20;
uint32_t stas_length = 1;
uint32_t stas_window = 10;
int stas_threshold = 5;         // pkts
string type = "discrete";
bool verbose = false;
int end_delay = 0;
bool non_zero_start = false;
double len = 9.8;               // meters
double wid = 7.4;               // meters

Ptr<FlowMonitor> monitor;
FlowMonitorHelper flowmon;
Scenario *wifiScenario;
deque<float> history;
deque<uint32_t> eff_stas_ax;
deque<uint32_t> eff_stas_legacy;

int payloadSize;
uint64_t g_rxPktNum = 0;
uint64_t g_txPktNum = 0;

int rxError = 0;
int rxOk = 0;

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

// Define observation space
Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
    current_time += envStepTime;

    float low = 0.0;
    float high = 10.0;
    std::vector<uint32_t> shape = {
        history_length,
    };
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    if (verbose)
        NS_LOG_UNCOND("MyGetObservationSpace: " << space);
    return space;
}

// Define action space
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

// Define extra info.
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
    static float ticks = 0.0;
    static float lastValueData = 0.0;
    float thr = g_rxDataPktNum - lastValueData; // Received data packets for throughput
    lastValueData = g_rxDataPktNum;
    ticks += envStepTime;

    float sentMbytes = thr * payloadSize * 8.0 / 1024 / 1024;

    std::string myInfo = std::to_string(sentMbytes);
    myInfo = myInfo + "|" + to_string(CW) + "|";
    myInfo = myInfo + to_string(CW_ax) + "|";
    myInfo = myInfo + to_string(wifiScenario->getActiveStationCount(ticks)) + "|";
    myInfo = myInfo + to_string(jain_index(addr));

    if (verbose)
        NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);

    return myInfo;
}

// Execute received actions
bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    if (verbose)
        NS_LOG_UNCOND("MyExecuteActions: " << action);

    Ptr<OpenGymBoxContainer<float>> box = DynamicCast<OpenGymBoxContainer<float>>(action);
    std::vector<float> actionVector = box->GetData();

    if (type == "discrete")
    {
        CW = pow(2, actionVector.at(0) + 4);
        CW_ax = pow(2, actionVector.at(1) + 4);
    }
    else if (type == "continuous")
    {
        CW = pow(2, actionVector.at(0) + 4) - 1; // add -1
        CW_ax = pow(2, actionVector.at(1) + 4) - 1; // add -1
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

    uint32_t min_cw = 15;
    uint32_t max_cw = 1023;

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

    float res = g_rxDataPktNum - last_packets;
    float reward = res * payloadSize * 8.0 / 1024 / 1024 / (5 * 150 * envStepTime) * 10;

    last_packets = g_rxDataPktNum;

    if (ticks <= 2 * envStepTime)
        return 0.0;

    if (verbose)
        NS_LOG_UNCOND("MyGetReward: " << reward);

    if(reward>1.0f || reward<0.0f)
        reward = last_reward;
    last_reward = reward;
    return last_reward;
}

// Collect observations
Ptr<OpenGymDataContainer> MyGetObservation()
{
    recordHistory();

    std::vector<uint32_t> shape = {
        history_length,
    };
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);

    // Copy p_col values
    for (uint32_t i = 0; i < history.size(); i++)
    {
        if (history[i] >= -100 && history[i] <= 100)
            box->AddValue(history[i]);
        else
            box->AddValue(0);
    }
    // Fill with 0s until history_length
    for (uint32_t i = history.size(); i < history_length; i++)
        box->AddValue(0);
    if (new_state)
    {
        // Copy effective legacy stas numbers
        for (uint32_t i = 0; i < eff_stas_legacy.size(); i++)
            box->AddValue(eff_stas_legacy[i]);
        // Fill with 0s until stas_length
        for (uint32_t i = eff_stas_legacy.size(); i < stas_length; i++)
            box->AddValue(0);
        // Copy effective ax stas numbers
        for (uint32_t i = 0; i < eff_stas_ax.size(); i++)
            box->AddValue(eff_stas_ax[i]);
        // Fill with 0s until stas_length
        for (uint32_t i = eff_stas_ax.size(); i < stas_length; i++)
            box->AddValue(0);
    }
    if (verbose)
        NS_LOG_UNCOND("MyGetObservation: " << box);
    return box;
}

bool MyGetGameOver(void)
{
    return false;
}

void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGymInterface)
{
    Simulator::Schedule(Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGymInterface);
    openGymInterface->NotifyCurrentState();
}

void recordHistory()
{
    static uint32_t last_ok = 0;
    static uint32_t last_error = 0;
    static uint32_t calls = 0;
    static uint32_t stas_win = stas_window;

    calls++;

    float ok = rxOk - last_ok;
    float error = rxError - last_error;
    float ratio;
    uint32_t stas_counter = 0;

    // p_col values
    ratio = 2*error/(2*error+ok/2);
    history.push_front(ratio);

    if (history.size() > history_length)
    {
        history.pop_back();
    }
    last_ok = rxOk;
    last_error = rxError;

    if (new_state)
    {
        // effective legacy stas numbers
        stas_win--;
        if (stas_win == 0)
        {
            for (uint32_t i = 0; i < effective_stas_legacy.size(); i++)
            {
                if (effective_stas_legacy[i] >= stas_threshold)
                    stas_counter++;
                effective_stas_legacy[i] = 0; // reset
            }
            eff_stas_legacy.push_front(stas_counter);
            if (eff_stas_legacy.size() > stas_length)
            {
                eff_stas_legacy.pop_back();
            }
            stas_win = stas_window;
        }
        // effective ax stas numbers
        eff_stas_ax.push_front(nAx);
        if (eff_stas_ax.size() > stas_length)
        {
            eff_stas_ax.pop_back();
        }
    }

    if (calls < history_length && non_zero_start)
    {
        Simulator::Schedule(Seconds(envStepTime), &recordHistory);
    }
    else if (calls == history_length && non_zero_start)
    {
        rxOk = 0;
        rxError = 0;
        last_ok = 0;
        last_error = 0;
    }
}

void phyRxOk(Ptr<const Packet> packet, double snr, WifiMode mode, WifiPreamble preamble)
{
    //NS_LOG_UNCOND("RX OK");
    rxOk++;
}

void phyRxError(Ptr<const Packet> packet, double snr)
{
    //NS_LOG_UNCOND("RX ERROR");
    rxError++;
}

void packetSent(std::string context, Ptr< const Packet > packet)
{
    // Get id of transmitter node
    std::string delimiter = "/";
    size_t pos = 0;
    std::string str_id;
    int id = -1;
    int counter = 0;
    while ((pos = context.find(delimiter)) != std::string::npos) {
        str_id = context.substr(0, pos);
        context.erase(0, pos + delimiter.length());
        if (counter == 2) // Reached node id
            break;
        counter++;
    }
    id = std::stoi(str_id);
    if (id != 0) // STA 0 emule 'nAx' STAs ax
        effective_stas_legacy[id]++;
}

void tcpDataPacketReceived(Ptr<const Packet> packet, const Address &ad)
{
    g_rxDataPktNum++;
}

void set_phy(int nWifi, int nAx, NodeContainer &staNodes, NodeContainer &apNode, YansWifiPhyHelper &phy)
{
    staNodes.Create(nWifi - nAx + 1);
    apNode.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    Ptr<YansWifiChannel> chan = channel.Create();

    Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel>();
    lossModel->SetDefaultLoss(50);

    chan->SetPropagationLossModel(lossModel);
    chan->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    phy = YansWifiPhyHelper::Default();
    phy.SetChannel(chan);
}

void set_nodes(int nWifi, int nAx, int channelWidth, int rng, int32_t simSeed, NodeContainer staNodes, NodeContainer apNode, Ipv4InterfaceContainer &stasInterface,
    Ipv4InterfaceContainer &apInterface, YansWifiPhyHelper phy, WifiMacHelper mac, WifiHelper wifi, NetDeviceContainer &apDevice, NetDeviceContainer &staDevice)
{
    Ssid ssid = Ssid("ns3-80211");

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
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(1.0, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install(apNode);
    mobility.Install(staNodes);

    // Internet stack
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

void set_sim(bool tracing, bool dry_run, int warmup, uint32_t openGymPort, YansWifiPhyHelper phy, NetDeviceContainer apDevice, int end_delay, Ptr<FlowMonitor> &monitor, 
    FlowMonitorHelper &flowmon, Address addr)
{
    monitor = flowmon.InstallAll();
    monitor->SetAttribute("StartTime", TimeValue(Seconds(warmup)));

    if (tracing)
    {
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.EnablePcap("pcap-" + std::to_string(std::time(0)), apDevice.Get(0));
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
        Simulator::Schedule(Seconds(envStepTime * history_length + 1.0), &ScheduleNextStateRead, envStepTime, openGymInterface);
    }
    else
        Simulator::Schedule(Seconds(1.0), &ScheduleNextStateRead, envStepTime, openGymInterface);

    Simulator::Stop(Seconds(simulationTime + end_delay + 1.0 + envStepTime*(history_length+1)));

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

    string outputCsv = "cw.csv";
    string scenario = "basic";
    bool tracing = true;
    bool useRts = false;
    int32_t simSeed = -1;
    uint32_t openGymPort = 5555;
    int rng = 1;
    int warmup = 1;

    int mcs = 11;
    int channelWidth = 20;	// MHz
    string dataRate = "150";    // Mbps
    int port = 1025;

    signal(SIGTERM, signalHandler);

    CommandLine cmd;
    cmd.AddValue("newState", "Use state = p_col + n (true) or state = p_col (false)", new_state);
    cmd.AddValue("udp", "UDP traffic if 'true', TCP if 'false'", udp);
    cmd.AddValue("uplink", "Uplink direction if 'true', downlink if 'false'", uplink);
    cmd.AddValue("nWifi", "Number of wifi 802.11ac STA devices", nWifi);
    cmd.AddValue("nAx", "Number of OFDMA 802.11ax STA devices", nAx);
    cmd.AddValue("scenario", "Scenario for analysis: basic, convergence", scenario);
    cmd.AddValue("dryRun", "Execute scenario with BEB and no agent interaction", dry_run);
    cmd.AddValue("CW", "Value of Contention Window for legacy devices", CW);
    cmd.AddValue("CW_ax", "Value of Contention Window for 802.11ax devices", CW_ax);
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
    cmd.AddValue("envStepTime", "Step time in seconds. Default: 0.1s", envStepTime);
    cmd.AddValue("historyLength", "Length of history window", history_length);
    cmd.AddValue("stasWindow", "Length of eff_stas window", stas_window);
    cmd.AddValue("stasThreshold", "Threshold to consider stas as active", stas_threshold);
    cmd.AddValue("agentType", "Type of agent actions: discrete, continuous", type);
    cmd.AddValue("seed", "Random seed", simSeed);
    cmd.AddValue("openGymPort", "Specify port number. Default: 5555", openGymPort);
    cmd.AddValue("rng", "Number of RngRun", rng);
    cmd.AddValue("nonZeroStart", "Start only after history buffer is filled", non_zero_start);

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
    NS_LOG_UNCOND("--udp: " << udp);
    NS_LOG_UNCOND("--uplink: " << uplink);
    NS_LOG_UNCOND("--simulationTime: " << simulationTime);
    NS_LOG_UNCOND("--openGymPort: " << openGymPort);
    NS_LOG_UNCOND("--envStepTime: " << envStepTime);
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
    set_phy(nWifi, nAx, staNodes, apNode, phy);

    WifiMacHelper mac;
    WifiHelper wifi;

    std::ostringstream oss;

    //TODO: configurar 802.11ax para STA 0 y 802.11ac o n para las demás STAs

    //802.11ax PHY
    phy.Set ("GuardInterval", TimeValue(NanoSeconds(800)));
    wifi.SetStandard(WIFI_PHY_STANDARD_80211ax_5GHZ);
    oss << "HeMcs" << mcs;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()), "ControlMode", StringValue(oss.str()));

    //802.11ac PHY
    //phy.Set ("ShortGuardEnabled", BooleanValue (true));
    //wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
    //oss << "VhtMcs" << mcs;
    //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (oss.str()), "ControlMode", StringValue(oss.str()));

    //802.11n PHY
    //phy.Set ("ShortGuardEnabled", BooleanValue (true));
    //wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
    //oss << "HtMcs" << mcs;
    //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (oss.str()), "ControlMode", StringValue (oss.str()));

    NetDeviceContainer apDevice;
    NetDeviceContainer staDevice;
    set_nodes(nWifi, nAx, channelWidth, rng, simSeed, staNodes, apNode, stasInterface, apInterface, phy, mac, wifi, apDevice, staDevice);

    ScenarioFactory helper = ScenarioFactory(nWifi, nAx, staNodes, apNode, stasInterface, apInterface, port, dataRate, history_length);
    wifiScenario = helper.getScenario(scenario);

    if (non_zero_start)
        end_delay = envStepTime * history_length + 1.0;
    else
        end_delay = 0.0;

    wifiScenario->installScenario(simulationTime + end_delay + envStepTime, envStepTime, uplink, udp, payloadSize);

    // Traces for p_col calculation
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback(&phyRxOk));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback(&phyRxError));

    // Trace for eff_stas
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&packetSent));

    // Trace for throughput calculation -- TCP
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback(&tcpDataPacketReceived));

    wifiScenario->PopulateARPcache();
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    set_sim(tracing, dry_run, warmup, openGymPort, phy, apDevice, end_delay, monitor, flowmon, apInterface.GetAddress(0));

    float res =  g_rxPktNum * payloadSize * 8.0 / 1024 / 1024;
    printf("Sent mbytes: %.2f\tThroughput: %.3f", res, res/simulationTime);

    Simulator::Destroy();
    NS_LOG_UNCOND("Packets registered by handler: " << g_rxPktNum << " Packets" << endl);

    return 0;
}
