from vehicle import MavlinkDrone
import time

def main():
    iha=MavlinkDrone()

    baglantı=iha.connect("udp:127.0.0.1:14550")

    if not baglantı ==True:
        print("Bağlantı başarısız")
        return
    time.sleep(2)

    print("motor açılıyor")
    iha.arm()
    time.sleep(2)

    hedef_yükseklik=10

    print(f"{hedef_yükseklik} m ye çıkılıyor")

    iha.takeoff(hedef_yükseklik)

    time.sleep(2)

    while True:
        veri=iha.get_telemetry_snapshot()
        anlık_yükseklik=veri["position"]["alt"]
        if anlık_yükseklik >= hedef_yükseklik * 0.95:
            print("hedefe ulaşıldı")
            break
        time.sleep(1)

    print("asılı kalınıyor")
    time.sleep(5)

    print("iniş")

    iha.land()

    while True:
        veri = iha.get_telemetry_snapshot()
        alt = veri["position"]["alt"]
        if alt < 0.5:
            print("görev tamamlandı")
            time.sleep(1)
            break

if __name__=="__main__":
        main()