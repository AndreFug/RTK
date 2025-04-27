#!/usr/bin/env python3
import senti_pb2 as senti
import sentipy
from math import radians, cos, pi
from datetime import datetime

# Earth radius for small‐angle approximation (WGS84)
R = 6378137.0

UDP_IP, UDP_PORT = "127.0.0.1", 21314

class RTKNavIMU:
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        # Initialize parser
        self.parser = sentipy.SentiMessageParser(ip, port)
        self.parser.connect()

        # Home reference for GNSS LLH
        self.origin_set = False
        self.lat0 = self.lon0 = self.h0 = 0.0

        # Store last IMU message
        self.last_imu = None

        # Register callbacks
        self.parser.register_callback(
            senti.MessageID.GNSS_STATUS, self.handle_status
        )
        self.parser.register_callback(
            senti.MessageID.GNSS_POS_LLH, self.handle_llh
        )
        self.parser.register_callback(
            senti.MessageID.IMU_MAG_ORIENTATION_MSG, self.handle_imu
        )

    def timestamp(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def handle_status(self, msg: senti.GNSSStatus):
        ts = self.timestamp()
        sats = msg.num_sv_used
        diff = 'Y' if msg.diff_corr else 'N'
        carr = 'T' if msg.carr_soln else 'F'
        fix = msg.fix_type
        fix_str = {
            senti.FixType.NO_FIX:    "NO_FIX",
            senti.FixType.GNSS_FIX:  "GNSS_FIX",
            senti.FixType.DGNSS_FIX: "DGNSS_FIX",
            senti.FixType.RTK_FLOAT: "RTK_FLOAT",
            senti.FixType.RTK_FIX:   "RTK_FIX",
        }.get(fix, str(fix))
        print(f"[{ts}] [GNSS STATUS] sats={sats} diff={diff} carr={carr} fix={fix_str}")

    def handle_llh(self, msg: senti.GNSSPosLLH):
        # Only use RTK_FIX LLH
        if msg.fix != senti.FixType.RTK_FIX:
            return

        ts = self.timestamp()
        lon_rad, lat_rad, h = msg.lon_lat_h
        lat = lat_rad * 180.0 / pi
        lon = lon_rad * 180.0 / pi

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

        # Print GNSS position
        print(f"[{ts}] [LOCAL RTK POS] North={north:.3f} m  East={east:.3f} m  Down={down:.3f} m")

        # Print the most recent IMU data alongside GNSS
        if self.last_imu:
            ang_vel, lin_acc, quat, temp = self.last_imu
            print(f"[{ts}] [IMU] AngVel=({ang_vel[0]:.3f}, {ang_vel[1]:.3f}, {ang_vel[2]:.3f})  "
                  f"LinAcc=({lin_acc[0]:.3f}, {lin_acc[1]:.3f}, {lin_acc[2]:.3f})  "
                  f"Orient=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})  "
                  f"Temp={temp:.2f}°C")
            # Clear stored IMU to avoid repeated prints if no new IMU arrives
            self.last_imu = None

    def handle_imu(self, msg: senti.IMUMagOrientationMsg):
        # Store the latest IMU data; do not print immediately
        self.last_imu = (msg.ang_vel, msg.lin_acc, msg.orientation, msg.temperature)

    def run(self):
        print(f"Listening on {UDP_IP}:{UDP_PORT} for GNSS & IMU (synced)... Ctrl+C to exit")
        try:
            while True:
                self.parser.receive_message()
        except KeyboardInterrupt:
            print("Exiting.")

if __name__ == "__main__":
    RTKNavIMU().run()
