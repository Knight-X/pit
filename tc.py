
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


std::string tostring(Ipv4Address addr) {
  std::ostringstream oss;
  addr.Print(oss);
  return oss.str();
}

Ipv4Address randomip(int a, int b, int c) {

  uint32_t addr = (a << 24) | (b << 16) | (c << 8);
  return Ipv4Address(addr);
}

std::string GenerateNonConflict(int a, int b, int c) {
    Ipv4Address newIp = randomip(a, b, c);
    std::string ip_s = tostring(newIp);

    return ip_s;
}

std::string get_ips(int a, int b, int c) {
  
  std::string _ip = GenerateNonConflict(a, b, c);
  return _ip;
}

void
TraceN0Rtt(double* rtt_n0, Time oldRtt, Time newRtt)
{
    *rtt_n0 = newRtt.GetSeconds();
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
ScheduleN0TcpRttTraceConnection(double* rtt_n0)
{
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/RTT",
                                  MakeBoundCallback(&TraceN0Rtt, rtt_n0));
}

class NetworkModel {
  public:
	  void init(int process_id);
      std::vector<double> step(int _quantum, int _target, int _maxsize);
	  void finish();
  private:
    Ptr<TrafficControlLayer> tc;
    Ptr<FqCoDelQueueDisc> fqCoDel;
    ApplicationContainer n0App;
    ApplicationContainer n4SinkApp;
    int ip_id;
    double rtt_n0;
    double throughput;
    double _loss_ratio;
    uint32_t g_n0BytesLoss;
    uint32_t g_n0BytesSent;
    int _bottle_rate;
    int base_rtt;
    int file_index = 0;

};

void NetworkModel::init(int process_id) {
    ip_id = process_id;
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> rtt_dis(60, 85);
    file_index = file_index + 1;
    rtt_n0 = 0.0;
    throughput = 0.0;
    _loss_ratio = 0.0;
    g_n0BytesLoss = 0;
    g_n0BytesSent = 0;

    base_rtt = rtt_dis(gen);
    Time baseRtt = MilliSeconds(base_rtt);

    std::uniform_int_distribution<> rate_dis(10, 101);
    _bottle_rate = rate_dis(gen);
    std::string bottle_rate = std::to_string(_bottle_rate) + "Mbps";
    std::cout << std::left << std::setw(15) << "initialize:" 
              << "loss: " << std::right << std::setw(8) << std::fixed << std::setprecision(3) << _loss_ratio
              << " | bottle rate: "  << std::setw(8) << std::fixed << std::setprecision(2) << bottle_rate 
              << " | rtt: " << std::setw(8) << std::fixed << std::setprecision(2) << baseRtt 
              << std::endl;

    std::uniform_real_distribution<> linkdata_dis(0.9, 1.1);
    std::string linkDataRate = std::to_string(linkdata_dis(gen)) + "Gbps";
    Time oneWayDelay = baseRtt / 2;

    DataRate bottleneckRate(bottle_rate);
    
    int _max_bytes_val = process_id * 100000000;



    ////////////////////////////////////////////////////////////
    // variables configured at command line                   //
    ////////////////////////////////////////////////////////////
    bool useCeThreshold = false;
    Time ceThreshold = MilliSeconds(1);
    std::string n0TcpType = "bic";
    bool useEcn = true;
    std::string queueType = "fq";
    uint32_t scenarioNum = 0;

    ////////////////////////////////////////////////////////////
    // Override ns-3 defaults                                 //
    ////////////////////////////////////////////////////////////
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1448));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(8192000));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(8192000));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType",
                       TypeIdValue(TcpPrrRecovery::GetTypeId()));

    TypeId n0TcpTypeId;
    TypeId queueTypeId;
    if (!scenarioNum)
    {
        if (useEcn)
        {
            Config::SetDefault("ns3::TcpSocketBase::UseEcn", StringValue("On"));
        }

        if (n0TcpType == "reno")
        {
            n0TcpTypeId = TcpNewReno::GetTypeId();
        }
        else if (n0TcpType == "bic")
        {
            n0TcpTypeId = TcpBic::GetTypeId();
        }
        else if (n0TcpType == "dctcp")
        {
            n0TcpTypeId = TcpDctcp::GetTypeId();
        }
        else
        {
            NS_FATAL_ERROR("Fatal error:  tcp unsupported");
        }


        if (queueType == "fq")
        {
            queueTypeId = FqCoDelQueueDisc::GetTypeId();
        }
        else if (queueType == "codel")
        {
            queueTypeId = CoDelQueueDisc::GetTypeId();
        }
        else
        {
            NS_FATAL_ERROR("Fatal error:  queueType unsupported");
        }
        if (useCeThreshold)
        {
            Config::SetDefault("ns3::FqCoDelQueueDisc::CeThreshold", TimeValue(ceThreshold));
        }
    }



    ////////////////////////////////////////////////////////////
    // scenario setup                                         //
    ////////////////////////////////////////////////////////////
    Ptr<Node> n0Server = CreateObject<Node>();
    Ptr<Node> n2 = CreateObject<Node>();
    Ptr<Node> n3 = CreateObject<Node>();
    Ptr<Node> n4Client = CreateObject<Node>();
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

  
    // Device containers
    NetDeviceContainer n0ServerDevices;
    NetDeviceContainer n2n3Devices;
    NetDeviceContainer n4ClientDevices;

    //set up the channel and device attribute among devices
    PointToPointHelper p2p;
    p2p.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("3p")));
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(linkDataRate)));
    p2p.SetChannelAttribute("Delay", TimeValue(oneWayDelay));
    n0ServerDevices = p2p.Install(n2, n0Server);

    // In scenario 9, base RTT of n1server (dctcp) is 1ms
    if (scenarioNum == 9)
    {
        p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(500)));
    }
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    n2n3Devices = p2p.Install(n2, n3);
    n4ClientDevices = p2p.Install(n3, n4Client);
    Ptr<PointToPointNetDevice> p = n2n3Devices.Get(0)->GetObject<PointToPointNetDevice>();
    p->SetAttribute("DataRate", DataRateValue(bottleneckRate));

    InternetStackHelper stackHelper;
    stackHelper.InstallAll();

    // Set the per-node TCP type here
    Ptr<TcpL4Protocol> proto;
    proto = n4Client->GetObject<TcpL4Protocol>();
    proto->SetAttribute("SocketType", TypeIdValue(n0TcpTypeId));
    proto = n0Server->GetObject<TcpL4Protocol>();
    proto->SetAttribute("SocketType", TypeIdValue(n0TcpTypeId));

    // InternetStackHelper will install a base TrafficControlLayer on the node,
    // but the Ipv4AddressHelper below will install the default FqCoDelQueueDisc
    // on all single device nodes.  The below code overrides the configuration
    // that is normally done by the Ipv4AddressHelper::Install() method by
    // instead explicitly configuring the queue discs we want on each device.
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

    Ipv4AddressHelper ipv4;
    std::string newIp = get_ips(10, 10, ip_id);
    ipv4.SetBase(newIp.c_str(), "255.255.255.0");
    Ipv4InterfaceContainer n0ServerIfaces = ipv4.Assign(n0ServerDevices);
    std::string newIp_b = get_ips(172, 16, ip_id);
    ipv4.SetBase(newIp_b.c_str(), "255.255.255.0");
    Ipv4InterfaceContainer n2n3Ifaces = ipv4.Assign(n2n3Devices);
    std::string newIp_c = get_ips(192, 168, ip_id);
    ipv4.SetBase(newIp_c.c_str(), "255.255.255.0");
    Ipv4InterfaceContainer n4ClientIfaces = ipv4.Assign(n4ClientDevices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();


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


    // Setup traces that can be hooked now
    Ptr<QueueDisc> qd;
    tc = n2n3Devices.Get(0)->GetNode()->GetObject<TrafficControlLayer>();
    qd = tc->GetRootQueueDiscOnDevice(n2n3Devices.Get(0));
    fqCoDel = DynamicCast<FqCoDelQueueDisc>(qd);
    Time throughputSamplingInterval = MilliSeconds(200);
    Time marksSamplingInterval = MilliSeconds(100);
    Simulator::Schedule(MilliSeconds(100),
                        &ScheduleN0TcpRttTraceConnection,
                        &rtt_n0);
    Simulator::Schedule(throughputSamplingInterval,
                        &TraceN0Throughput,
                        throughputSamplingInterval,
			fqCoDel, &throughput, &_loss_ratio, &g_n0BytesSent, &g_n0BytesLoss);

    Time _now = Simulator::Now();
    Time _start = _now;
    Time stopTime = _now + Seconds(60);
    n0App.Start(_start);
    n0App.Stop(stopTime - Seconds(1));
    n4SinkApp.Start(_start);
    n4SinkApp.Stop(stopTime - MilliSeconds(500));
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
    std::vector<double> ret;
    ret.push_back(_loss_ratio); 
    ret.push_back(throughput);
    ret.push_back(rtt_n0);
    ret.push_back(_bottle_rate);
    ret.push_back(base_rtt);
    std::cout << std::setw(15) << std::left << "Step: " 
              << std::setw(10) << std::right <<  ip_id 
              << std::setw(10) << std::right << std::fixed << std::setprecision(2)  << _loss_ratio 
              << std::setw(10) << std::right << std::fixed << std::setprecision(2)  << throughput 
              << std::setw(10) << std::right << std::fixed << std::setprecision(2)  << rtt_n0
              << std::endl;
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

