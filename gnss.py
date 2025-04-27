#!/usr/bin/env python3
import senti_pb2 as senti
import sentipy
from math import radians, cos, pi
from datetime import datetime

# Earth radius for small‐angle approx (WGS84)
R = 6378137.0

UDP_IP, UDP_PORT = "127.0.0.1", 21314

class RTKNav:
    def __init__(self):
        # Connect parser to SentiBoard
        self.parser = sentipy.SentiMessageParser(UDP_IP, UDP_PORT)
        self.parser.connect()

        # State for home reference
        self.origin_set = False
        self.lat0 = self.lon0 = self.h0 = 0.0

        # Register callbacks
        self.parser.register_callback(
            senti.MessageID.GNSS_STATUS, self.handle_status
        )
        self.parser.register_callback(
            senti.MessageID.GNSS_POS_LLH, self.handle_llh
        )

    def handle_status(self, msg: senti.GNSSStatus):
        # Include timestamp to track dropouts
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        sats = msg.num_sv_used
        diff = 'Y' if msg.diff_corr else 'N'
        carr = 'T' if msg.carr_soln else 'F'
        fix  = msg.fix_type
        fix_str = {
            senti.FixType.NO_FIX:    "NO_FIX",
            senti.FixType.GNSS_FIX:  "GNSS_FIX",
            senti.FixType.DGNSS_FIX: "DGNSS_FIX",
            senti.FixType.RTK_FLOAT: "RTK_FLOAT",
            senti.FixType.RTK_FIX:   "RTK_FIX",
        }[fix]
        print(f"[{ts}] [GNSS STATUS] sats={sats} diff={diff} carr={carr} fix={fix_str}")

    def handle_llh(self, msg: senti.GNSSPosLLH):
        # Only proceed on full RTK-FIX
        if msg.fix != senti.FixType.RTK_FIX:
            return

        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        # Unpack & convert to degrees
        lon_rad, lat_rad, h = msg.lon_lat_h
        lat = lat_rad * 180.0 / pi
        lon = lon_rad * 180.0 / pi

        # On first fix, set home
        if not self.origin_set:
            self.lat0, self.lon0, self.h0 = lat, lon, h
            self.origin_set = True
            print(f"[{ts}] [HOME] lat={lat:.7f} lon={lon:.7f} h={h:.3f} m")
            return

        # Compute local N/E/D
        d_lat = lat - self.lat0
        d_lon = lon - self.lon0
        north = radians(d_lat) * R
        east  = radians(d_lon) * R * cos(radians(self.lat0))
        down  = h - self.h0

        print(f"[{ts}] [LOCAL RTK POS] North={north:.3f} m  East={east:.3f} m  Down={down:.3f} m")

    def run(self):
        print("Waiting for RTK-FIX LLH… Ctrl+C to exit")
        try:
            while True:
                self.parser.receive_message()
        except KeyboardInterrupt:
            print("Exiting.")

if __name__ == "__main__":
    RTKNav().run()

