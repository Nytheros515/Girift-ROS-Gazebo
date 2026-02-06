from vehicle import MavlinkDrone
import time

def main():
    iha = MavlinkDrone()
    
    baglantı = iha.connect("udp:127.0.0.1:14550")

    print("Bağlantı kuruluyor")

    if not baglantı == True:
        print("bağlantı başarısız")
        return
    print("Bağlantı kuruldu")

    time.sleep(1)

    

    iha.arm()
    print("Motorlar açılıyor")
    print("ADIM 1 5 METRE")
    time.sleep(2)
    iha.takeoff(5)

    while True:
        veri = iha.get_telemetry_snapshot()
        alt = veri["position"]["alt"]
        if alt >= 5 * 0.95:
            print("5 metreye ulaşıldı")
            break
        time.sleep(1)

    print("5 METREDE BEKLENİYOR")
    time.sleep(2)

    konum = iha.get_telemetry_snapshot()
    lat = konum["position"]["lat"]
    lon = konum["position"]["lon"]
    iha.goto_gps_location(lat, lon, 10)

    while True:
         veri = iha.get_telemetry_snapshot()
         alt = veri["position"]["alt"]
         if alt >= 10 * 0.95:
            print("10 metreye ulaşıldı")
            break
         time.sleep(1)

    print("10 METREDE BEKLENİYOR")
    time.sleep(2)

    konum = iha.get_telemetry_snapshot()
    lat = konum["position"]["lat"]
    lon = konum["position"]["lon"]
    iha.goto_gps_location(lat, lon, 15)

    while True:
         veri = iha.get_telemetry_snapshot()
         alt = veri["position"]["alt"]
         if alt >= 15 * 0.95:
            print("15 metreye ulaşıldı")
            break
         time.sleep(1)

    print("15 METREDE BEKLENİYOR")
    time.sleep(2)

    iha.land()

    while True:
        veri=iha.get_telemetry_snapshot()
        yükseklik=veri["position"]["alt"]
        if yükseklik < 0.5:
            print("GÖREV TAMAMLANDI")
            time.sleep(1)
            break
if __name__ == "__main__":
    main()


    

    


