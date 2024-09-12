
from ns import ns


ns.cppyy.cppdef(
    """
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include <iomanip>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FqCoDelSimulation");


void
TraceN0Rtt(std::vector<double> *rtt_vec, Time oldRtt, Time newRtt)
{
    double rtt_n0 = newRtt.GetSeconds();
    (*rtt_vec).push_back(rtt_n0);
}


void
TracePingRtt(double* rtt_ping, uint16_t seqNo, Time rtt)
{
    *rtt_ping = rtt.GetSeconds();
}


double avg_rtt(std::vector<double>* rtt_vec) {
  if ((*rtt_vec).empty()) {
   return 0.0;
  }


  double sum = std::accumulate((*rtt_vec).begin(), (*rtt_vec).end(), 0.0);

  double avg = sum / (*rtt_vec).size();


  return avg;
}

void
TraceN0Throughput(Time throughputInterval, Ptr<FqCoDelQueueDisc> a, double* throughput, double *_loss_ratio, uint32_t *g_n0BytesSent, uint32_t *g_n0BytesLoss)
{
    uint32_t total_sent = a->GetStats().nTotalSentBytes;
    uint32_t total_loss = a->GetStats().nTotalDroppedBytes;
    uint32_t _loss = total_loss - *g_n0BytesLoss;
    *_loss_ratio = static_cast<double>(_loss * 8) / throughputInterval.GetSeconds() / 1e6; 
    uint32_t _sent = total_sent - *g_n0BytesSent;
    *throughput = static_cast<double>(_sent * 8) / throughputInterval.GetSeconds() / 1e6;
    Time _now = Simulator::Now();
    *g_n0BytesSent = total_sent;
    *g_n0BytesLoss = total_loss;
    Simulator::Schedule(throughputInterval, &TraceN0Throughput, throughputInterval, a, throughput, _loss_ratio, g_n0BytesSent, g_n0BytesLoss);
}



void
ScheduleN0TcpRttTraceConnection(std::vector<double>* rtt_vec)
{
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/RTT",
                                  MakeBoundCallback(&TraceN0Rtt, rtt_vec));
}

int fluctuate(int value, int percentage = 5) {
  static std::mt19937 gen(std::time(0));

  double fluctuation_range = value * (percentage / 100.0);

  std::uniform_real_distribution<> dis(-fluctuation_range, fluctuation_range);

  return std::round(value + dis(gen));
}

class NetworkModel {
  public:
	  void init(int process_id);
      void addPing(Ptr<Node> n3, Time _start, Time stopTime);
      std::vector<double> step(int _quantum, int _target, int _maxsize);
	  void finish();
  private:

    void configureConnection();
    void configureInternetStack();
    void setupAddress();
    void setupApplications();
    void setupTraces();


    Ptr<Node> n0Server, n2, n3, n4Client;
    NetDeviceContainer n0ServerDevices, n2n3Devices, n4ClientDevices;
    Ipv4InterfaceContainer n0ServerIfaces, n2n3Ifaces, n4ClientIfaces;
    Ptr<TrafficControlLayer> tc;
    Ptr<FqCoDelQueueDisc> fqCoDel;
    ApplicationContainer n0App;
    ApplicationContainer n4SinkApp;
    ApplicationContainer apps;
    int ip_id;
    double throughput;
    double _loss_ratio;
    double rtt_ping;
    uint32_t g_n0BytesLoss;
    uint32_t g_n0BytesSent;
    int _bottle_rate;
    int base_rtt;
    double link_data_rate; 
    std::vector<double> rtt_vec;


};

void NetworkModel::addPing(Ptr<Node> n3, Time _start, Time stopTime) {
    uint32_t size{10000};
    uint32_t count{60};
    Time interPacketInterval{Seconds(4.0)};
    PingHelper pingHelper(Ipv4Address("192.168.1.2"));
    pingHelper.SetAttribute("Interval", TimeValue(interPacketInterval));
    pingHelper.SetAttribute("Size", UintegerValue(size));
    pingHelper.SetAttribute("Count", UintegerValue(count));
    apps = pingHelper.Install(n3);
    Ptr<Ping> ping = apps.Get(0)->GetObject<Ping>();
    ping->TraceConnectWithoutContext("Rtt", MakeBoundCallback(&TracePingRtt, &rtt_ping));
    apps.Start(_start);
    apps.Stop(stopTime - Seconds(1));

}

void NetworkModel::init(int process_id) {
    ip_id = process_id;
    std::random_device rd; 
    std::mt19937 gen(rd());
    rtt_ping = 0.0;
    throughput = 0.0;
    _loss_ratio = 0.0;
    g_n0BytesLoss = 0;
    g_n0BytesSent = 0;


    std::uniform_int_distribution<> rtt_dis(60, 85);
    base_rtt = rtt_dis(gen);

    std::uniform_int_distribution<> rate_dis(10, 101);
    _bottle_rate = rate_dis(gen);

    std::cout << std::left << std::setw(15) << "initialize:" 
              << "loss: " << std::right << std::setw(8) << std::fixed << std::setprecision(3) << _loss_ratio
              << " | bottle rate: "  << std::setw(8) << std::fixed << std::setprecision(2) << _bottle_rate 
              << " | rtt: " << std::setw(8) << std::fixed << std::setprecision(2) << base_rtt 
              << std::endl;

    std::uniform_real_distribution<> linkdata_dis(0.9, 1.1);
    link_data_rate = linkdata_dis(gen);

    
    bool useEcn = true;

    ////////////////////////////////////////////////////////////
    // Override ns-3 defaults                                 //
    ////////////////////////////////////////////////////////////
    int _segment_size = fluctuate(1448);
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(_segment_size));
    int _sendbuf_size = fluctuate(8192000);
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(_sendbuf_size));
    int _rcvbuf_size = fluctuate(8192000);
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(_rcvbuf_size));
    int _cwnd = fluctuate(10, 10);
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(_cwnd));
    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType",
                       TypeIdValue(TcpPrrRecovery::GetTypeId()));

    if (useEcn)
    {
        Config::SetDefault("ns3::TcpSocketBase::UseEcn", StringValue("On"));
    }

    ////////////////////////////////////////////////////////////
    // scenario setup                                         //
    ////////////////////////////////////////////////////////////
    n0Server = CreateObject<Node>();
    n2 = CreateObject<Node>();
    n3 = CreateObject<Node>();
    n4Client = CreateObject<Node>();
    MobilityHelper mobility;

    //make the position of the router randomly
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(1.0),
                                  "MinY",
                                  DoubleValue(1.0),
                                  "DeltaX",
                                  DoubleValue(5.0),
                                  "DeltaY",
                                  DoubleValue(5.0),
                                  "GridWidth",
                                  UintegerValue(3),
                                  "LayoutType",
                                  StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Mode",
                              StringValue("Time"),
                              "Time",
                              StringValue("2s"),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "Bounds",
                              RectangleValue(Rectangle(0.0, 20.0, 0.0, 20.0)));
    mobility.Install(n4Client);

    AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("mobility-trace-example.mob"));

    configureConnection();
    configureInternetStack();
    setupAddress();
    setupApplications();
    setupTraces();

}

void NetworkModel::configureConnection() { 
    Time baseRtt = MilliSeconds(base_rtt);
    Time oneWayDelay = baseRtt / 2;
    std::string bottle_rate = std::to_string(_bottle_rate) + "Mbps";
    DataRate bottleneckRate(bottle_rate);
    std::string linkDataRate = std::to_string(link_data_rate) + "Gbps";


    //set up the channel and device attribute among devices
    PointToPointHelper p2p;
    p2p.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("3p")));
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(linkDataRate)));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    n0ServerDevices = p2p.Install(n2, n0Server);

    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    n2n3Devices = p2p.Install(n2, n3);
    p2p.SetChannelAttribute("Delay", TimeValue(oneWayDelay));
    n4ClientDevices = p2p.Install(n3, n4Client);
    Ptr<PointToPointNetDevice> p = n2n3Devices.Get(0)->GetObject<PointToPointNetDevice>();
    p->SetAttribute("DataRate", DataRateValue(bottleneckRate));

}

void NetworkModel::configureInternetStack() {
    TypeId n0TcpTypeId = TcpBic::GetTypeId();
    TypeId queueTypeId = FqCoDelQueueDisc::GetTypeId();

    InternetStackHelper stackHelper;
    stackHelper.InstallAll();

    // Set the per-node TCP type here
    Ptr<TcpL4Protocol> proto;
    proto = n4Client->GetObject<TcpL4Protocol>();
    proto->SetAttribute("SocketType", TypeIdValue(n0TcpTypeId));
    proto = n0Server->GetObject<TcpL4Protocol>();
    proto->SetAttribute("SocketType", TypeIdValue(n0TcpTypeId));

    // Configure queue qdisc
    TrafficControlHelper tchFq;
    tchFq.SetRootQueueDisc("ns3::FqCoDelQueueDisc");
    tchFq.SetQueueLimits("ns3::DynamicQueueLimits", "HoldTime", StringValue("1ms"));
    tchFq.Install(n0ServerDevices);
    tchFq.Install(n2n3Devices.Get(1)); // n2 queue for bottleneck link
    tchFq.Install(n4ClientDevices);

    TrafficControlHelper tchN2;
    tchN2.SetRootQueueDisc(queueTypeId.GetName());
    tchN2.SetQueueLimits("ns3::DynamicQueueLimits", "HoldTime", StringValue("1000ms"));
    tchN2.Install(n2n3Devices.Get(0));

}

void NetworkModel::setupAddress() {
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.10.13.0", "255.255.255.0");
    n0ServerIfaces = ipv4.Assign(n0ServerDevices);
    ipv4.SetBase("172.16.10.0", "255.255.255.0");
    n2n3Ifaces = ipv4.Assign(n2n3Devices);
    ipv4.SetBase("192.168.1.0", "255.255.255.0");
    n4ClientIfaces = ipv4.Assign(n4ClientDevices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
}

void NetworkModel::setupApplications() {
    int _max_bytes_val = ip_id * 100000000;
    BulkSendHelper tcp("ns3::TcpSocketFactory", Address());

    // set to large value:  e.g. 1000 Mb/s for 60 seconds = 7500000000 bytes
    tcp.SetAttribute("MaxBytes", UintegerValue(_max_bytes_val));

    // Configure n4/n0 TCP client/server pair
    uint16_t n4Port = 5000;
    InetSocketAddress n0DestAddress(n4ClientIfaces.GetAddress(1), n4Port);
    tcp.SetAttribute("Remote", AddressValue(n0DestAddress));
    n0App = tcp.Install(n0Server);

    Address n4SinkAddress(InetSocketAddress(Ipv4Address::GetAny(), n4Port));
    PacketSinkHelper n4SinkHelper("ns3::TcpSocketFactory", n4SinkAddress);
    n4SinkApp = n4SinkHelper.Install(n4Client);


    Time _now = Simulator::Now();
    Time _start = _now;
    Time stopTime = _now + Seconds(60);
    n0App.Start(_start);
    n0App.Stop(stopTime - Seconds(1));
    n4SinkApp.Start(_start);
    n4SinkApp.Stop(stopTime - MilliSeconds(500));

    addPing(n3, _start, stopTime);
}

void NetworkModel::setupTraces() {
    // Setup traces that can be hooked now
    Ptr<QueueDisc> qd;
    tc = n2n3Devices.Get(0)->GetNode()->GetObject<TrafficControlLayer>();
    qd = tc->GetRootQueueDiscOnDevice(n2n3Devices.Get(0));
    fqCoDel = DynamicCast<FqCoDelQueueDisc>(qd);
    Time throughputSamplingInterval = MilliSeconds(200);
    Time marksSamplingInterval = MilliSeconds(100);
    Simulator::Schedule(MilliSeconds(100),
                        &ScheduleN0TcpRttTraceConnection,
                        &rtt_vec);
    Simulator::Schedule(throughputSamplingInterval,
                        &TraceN0Throughput,
                        throughputSamplingInterval,
			fqCoDel, &throughput, &_loss_ratio, &g_n0BytesSent, &g_n0BytesLoss);

}

std::vector<double> NetworkModel::step(int _quantum, int _target, int _maxsize) {
    Time _now = Simulator::Now();
    fqCoDel->SetQuantum(_quantum);
    std::string target_val = std::to_string(_target) + "ms"; 
    fqCoDel->SetAttribute("Target", StringValue(target_val.c_str()));
    std::string _queuesize = std::to_string(_maxsize) + "p";
    fqCoDel->SetMaxSize(QueueSize(_queuesize.c_str()));
    Simulator::Stop(Seconds(5));
   	Simulator::Run();
    double rtt_n0 = avg_rtt(&rtt_vec);
    std::vector<double> ret;
    ret.push_back(_loss_ratio); 
    ret.push_back(throughput);
    ret.push_back(rtt_n0);
    ret.push_back(_bottle_rate);
    ret.push_back(base_rtt);
    std::cout << std::setw(15) << std::left << "Step: " 
              << std::setw(10) << std::right <<  ip_id 
              << std::setw(10) << std::right << std::fixed << std::setprecision(3)  << _loss_ratio 
              << std::setw(10) << std::right << std::fixed << std::setprecision(3)  << throughput 
              << std::setw(10) << std::right << std::fixed << std::setprecision(3)  << rtt_n0
              << std::setw(10) << std::right << std::fixed << std::setprecision(3)  << rtt_ping
              << std::endl;
    rtt_vec.clear();
    return ret;
}

void NetworkModel::finish() {
    Simulator::Destroy();

}
    """
    )
    

class ns_sim:
    def __init__(self, _count):
        self.model = ns.cppyy.gbl.NetworkModel()
        self._count = _count

    def step(self, action):
        ret = self.model.step(int(action[0]), int(action[1]), int(action[2]))
        return ret

    def reset(self, seed):
        print("reset {}".format(seed))
        self.model.finish()
        _count = int(self._count)
        self.model.init(_count)

