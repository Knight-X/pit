from pyroute2 import IPRoute
from stable_baselines3 import PPO
import time
import subprocess
from ping3 import ping 


def ping_ip(ip, count=5):
    """Ping an IP address multiple times using ping3 and return the average latency and loss ratio."""
    latencies = []
    for _ in range(count):
        result = ping(ip, timeout=0.8)
        if result:
            print(result)
            latencies.append(result * 1000)  # Convert to milliseconds
        #time.sleep(0.2)  # Small delay between pings
    
    received = len(latencies)

    avg_latency = sum(latencies) / received if received > 0 else None
    return ip, avg_latency

def set_traffic_control(ip, interface, target, quantum, limit):
    index = ip.link_lookup(ifname=interface)
    index = index[0]

    # Remove existing qdiscs
    qdiscs = ip.get_qdiscs(index)
    cmd = f"sudo tc qdisc replace dev {interface} root fq_codel limit {limit} quantum {quantum} target {target}"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
    else:
        print("success")
        
def get_interface_stats(ip, interface):
    index = ip.link_lookup(ifname=interface)[0]
    links = ip.get_links(index)

    if links:
        stats = links[0].get_attr('IFLA_STATS64') or links[0].get_attr('IFLA_STATS')
        return stats
    return None

def get_tc_stats(ip, interface):
    index = ip.link_lookup(ifname=interface)
    index = index[0]
    stats = get_interface_stats(ip, interface)
    qdiscs = ip.get_qdiscs(index)
    
    qdisc_stats = next((q for q in qdiscs if q.get('kind') == 'fq_codel'), None)
    return {
        'bytes': stats.get('tx_bytes', 0),
        'packets': stats.get('tx_packets', 0),
        'tx_dropped': stats.get('tx_dropped', 0),
    }


def calculate_throughput_loss(start_bytes, end_bytes, duration, start_dropped, end_dropped):
    bytes_diff = end_bytes - start_bytes
    bits_diff = bytes_diff * 8
    throughput_mbps = (bits_diff / duration) / 1_000_000
    dropped_diff = end_dropped - start_dropped
    d_bits_diff = dropped_diff * 8
    dropped_mbps = (d_bits_diff / duration) / 1_000_000
    return throughput_mbps, dropped_mbps

def latency(_ip):
    _latency = ping(_ip)
    return _latency
    
def main():
    interface = "wlan0"  # Replace with your network interface
    measurement_duration = 5  # seconds
    _ip = "192.168.1.105"
    ip = IPRoute()
    model = PPO.load("ppo_stand_old")
    quantum = 0
    target = 0
    limit = 0
    def scale_action(action, low, high):
        return low + (action + 1) * 0.5 * (high - low)
    try:
        
        for i in range(1):
            # Get initial statistics
            start_stats = get_tc_stats(ip, interface)
            start_time = time.time()
        
            print(f"Measuring for {measurement_duration} seconds...")
            _, _latency= ping_ip(_ip)
        
            # Get final statistics
            end_stats = get_tc_stats(ip, interface)
            end_time = time.time()
        
            # Calculate actual duration
            actual_duration = end_time - start_time
        
            # Calculate throughput
            throughput, loss = calculate_throughput_loss(start_stats['bytes'], end_stats['bytes'], actual_duration, start_stats['tx_dropped'], end_stats['tx_dropped'])
            

            obs = [loss, throughput, _latency / 1000, quantum, target, limit]
            action, _states = model.predict(obs, deterministic=True)
            action[0] = scale_action(action[0], 500, 9000)
            action[1] = scale_action(action[1], 3, 1000)
            action[2] = scale_action(action[2], 10000, 1000000)
            quantum = action[0]
            target = action[1]
            limit = action[2]
            
            #limit action space
            if action[0] < 500:
                quantum = 500
            elif action[0] > 9000:
                quantum = 9000
                

            if action[1] < 3:
                target = 3
            elif action[1] > 1000:
                target = 1000

            if action[2] < 10000:
                limit = 10000
            elif action[2] > 1000000:
                limit = 10000000
            set_traffic_control(ip, interface, quantum, target, limit)
            print(f"\nTC Statistics:")
            print(f"latency : {_latency}, loss {loss}")
            print(f"Measured Throughput: {throughput:.2f} Mbps")
            print(f"Packets Sent: {end_stats['packets'] - start_stats['packets']}")
            print(f"Bytes Sent: {end_stats['bytes'] - start_stats['bytes']}")
            
         
    finally:
        ip.close()


if __name__ == "__main__":
    main()

