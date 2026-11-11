"""Observation-only CLI. No motor commands are exposed here."""
import argparse
from dataclasses import asdict
import json
import time
from wifistepper_uart import Host


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("port", help="verified Pi UART device, e.g. /dev/serial0")
    p.add_argument("--seconds", type=float, default=10)
    p.add_argument("--log", help="write timestamped raw TX/RX chunks as JSONL")
    p.add_argument("--passive", action="store_true", help="receive only; no discovery ping")
    args=p.parse_args()
    logfile=open(args.log,"w") if args.log else None
    def trace(kind, stamp, data):
        if logfile:
            logfile.write(json.dumps({"direction":kind,"monotonic":stamp,"hex":data.hex()})+"\n")
            logfile.flush()
    try:
        with Host.open_serial(args.port,trace=trace) as host:
            if not args.passive:
                count=host.discover()
                print(f"Discovered {count} slave(s)")
                for target in range(1,count+1):
                    identity=host.identify(target)
                    print(target,identity.product,identity.model,identity.branch,identity.version)
            deadline=time.monotonic()+args.seconds
            last={}
            while time.monotonic()<deadline:
                host.poll()
                for target,(stamp,state) in host.states.items():
                    if last.get(target)!=stamp:
                        print(json.dumps({"target":target,"monotonic":stamp,**asdict(state)},
                                         default=lambda x:x.hex()))
                        last[target]=stamp
                time.sleep(.001)
            print("Parser:", {name:getattr(host.parser,name) for name in
                              ("bad_headers","bad_bodies","expired","discarded")})
            print("Diagnostics:",list(host.diagnostics))
    finally:
        if logfile: logfile.close()

if __name__=="__main__": main()
