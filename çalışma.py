from vehicle import MavlinkDrone
import time

def main():
    iha = MavlinkDrone()
    
    # ADIM 1
    print("1. Bağlanılıyor...")
    baglanti = iha.connect("udp:127.0.0.1:14550")
    
    if baglanti:
        print("2. Bağlantı BAŞARILI!")
    else:
        print("HATA: Bağlanamadı")
        return

    time.sleep(1)
    
    # ADIM 3
    print("3. Veri isteniyor...")
    
    # Buraya koruma (try-except) koyuyoruz ki hatayı görelim
    try:
        veriler = iha.get_telemetry_snapshot()
        print("4. Veri paketi geldi! İçini açıyoruz...")
        print(f"   HAM VERİ: {veriler}") # Gelen veriyi görelim, belki boştur?

        # ADIM 5
        pil = veriler["battery"]["voltage"]
        print(f"5. Pil okundu: {pil}")
        
        yukseklik = veriler["position"]["alt"]
        print(f"6. Yükseklik okundu: {yukseklik}")

    except Exception as hata_mesaji:
        print("\n🚨 HATA YAKALANDI! 🚨")
        print(f"Sorun tam olarak şu: {hata_mesaji}")
        print("Muhtemelen veri boş (None) geliyor.")

if __name__ == "__main__":
    main()