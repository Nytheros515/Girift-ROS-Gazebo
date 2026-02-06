from vehicle import MavlinkDrone
from pymavlink import mavutil
import time

def donus_yap(iha ,derece ,hiz=0, yon=0, relative=0):
    


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
    baglantı = iha.connect("udp:127.0.0.1:14550")
    print("bağlantı kuruluyor")
    if not baglantı == True:
        print("bağlantı yok")
        return
    time.sleep(2)
    print("bağlantı kuruldu")

    iha.arm()
    print("motorlar açılıyor")
    time.sleep(2)

    print("tırmanma başlıyor")
    iha.takeoff(10)
    time.sleep(2)

    hedef_yükseklik = 10

    while True:
        veriler = iha.get_telemetry_snapshot()
        konum = veriler["position"]["alt"]
        if konum >= hedef_yükseklik * 0.95:
            print("istenen irtifaya ulaşıldı")
            break

    time.sleep(1)

    print("ikinci adım")

    donus_yap(iha ,0 ,relative=0)
    time.sleep(5)

    veriler= iha.get_telemetry_snapshot()
    lon = veriler["position"]["lon"]
    lat = veriler["position"]["lat"]
    hedef_lat = lat + 0.0001
    konum = veriler["position"]["alt"]
    iha.goto_gps_location(hedef_lat, lon , konum)
    time.sleep(2)
    print("ikinci  adım başarılı")

    

    while True:
        veriler = iha.get_telemetry_snapshot()
        lat = veriler["position"]["lat"]
        fark = abs(hedef_lat - lat)
        if  fark <= 0.00001:
            print("istenen lat a  ulaşıldı")
            break
    
    time.sleep(2)
    print("3.adıma geçiliyor")

    donus_yap(iha,90, relative=0)

    veriler = iha.get_telemetry_snapshot()
    lon = veriler["position"]["lon"]
    lat = veriler["position"]["lat"]
    hedef_lon = lon + 0.0001
    konum = veriler["position"]["alt"]
    iha.goto_gps_location(lat ,hedef_lon ,konum)
    time.sleep(2)

    while True:
        veriler = iha.get_telemetry_snapshot()
        lon = veriler["position"]["lon"]
        fark = abs(hedef_lon - lon)
        if fark <= 0.00001:
            print("istenen lon a ulaşıldı")
            break

    time.sleep(2)
    print("4.adıma geçiliyor")

    donus_yap(iha,180, relative=0)

    veriler = iha.get_telemetry_snapshot()
    lon = veriler["position"]["lon"]
    lat = veriler["position"]["lat"]
    hedef_lat_iki = lat - 0.0001
    konum = veriler["position"]["alt"]
    iha.goto_gps_location(hedef_lat_iki ,lon ,konum)
    time.sleep(2)

    while True:
        veriler=iha.get_telemetry_snapshot()
        lat =veriler["position"]["lat"]
        fark = abs(hedef_lat_iki - lat)
        if fark <= 0.00001:
            print("istenen ikinci lat a ulaşıldı")
            break
    time.sleep(2)

    print("5. adıma geçiliyor")

    donus_yap(iha,270, relative=0)

    veriler = iha.get_telemetry_snapshot()
    lon = veriler["position"]["lon"]
    lat = veriler["position"]["lat"]
    hedef_lon_iki = lon - 0.0001
    konum = veriler["position"]["alt"]
    iha.goto_gps_location(lat ,hedef_lon_iki ,konum)

    time.sleep(2)
    while True:
        veriler=iha.get_telemetry_snapshot()
        lon = veriler["position"]["lon"]
        fark = abs(hedef_lon_iki - lon)
        if fark <= 0.00001:
            print("kare tamamlandı başarılı") 
            break
    time.sleep(2)

    time.sleep(3)

    print("eve dönülüyor")


    iha.rtl()






    time.sleep(1)
if __name__ == "__main__":
    main()
    





