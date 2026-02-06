from vehicle import MavlinkDrone
from pymavlink import mavutil
import time

def donus_yap(iha ,derece ,hiz=0, yon=1, relative=1):
    


    print(f"{derece}")    

    

    
    iha.vehicle.mav.command_long_send(
        iha.vehicle.target_system,
        iha.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        derece,
        hiz,
        yon,
        relative,
        0, 0, 0

    )

def main():
    iha = MavlinkDrone()

    iha.connect("udp:127.0.0.1:14550")

    iha.vehicle.mav.set_mode_send(
        iha.vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4
    )
    time.sleep(1)

    iha.arm()

    time.sleep(1)

    iha.takeoff(5)

    time.sleep(2)

    while True:
        veri = iha.get_telemetry_snapshot()
        if veri["position"]["alt"] >= 4.8:
            break
        time.sleep(2)


    time.sleep(2)


    ## DÖNÜŞ TESTİ




    donus_yap(iha, 90, relative=1)
    time.sleep(5)


    donus_yap(iha, 180,relative=1)
    time.sleep(5)

    donus_yap(iha, 0, relative=0)
    time.sleep(5)


    iha.rtl()
if __name__=="__main__":
    main()
