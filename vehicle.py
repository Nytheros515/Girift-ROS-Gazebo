# mavlink_drone.py
# GİRİFT İHA TAKIMI - MAVLink Kontrol Kütüphanesi (FİNAL OPTİMİZE)
# Bu dosya, İHA (İnsansız Hava Aracı) ile iletişim kurmak için gerekli tüm fonksiyonları içerir.

# pymavlink kütüphanesini içeri aktar: İHA ile MAVLink protokolü üzerinden iletişim kurmak için kullanılır
from pymavlink import mavutil
# time kütüphanesini içeri aktar: Zamanlamalar ve gecikmeler için kullanılır
import time
# logging kütüphanesini içeri aktar: Program hata ve bilgi mesajlarını loglamak için kullanılır
import logging
# math kütüphanesini içeri aktar: Matematiksel işlemler (açı dönüşümleri vb.) için kullanılır
import math
# json kütüphanesini içeri aktar: Veri dosyasını JSON formatında kaydetmek için kullanılır
import json
# threading kütüphanesini içeri aktar: Arka planda veri okuma gibi eş zamanlı işlemler için kullanılır
import threading
# typing kütüphanesini içeri aktar: Fonksiyon parametrelerinin türlerini belirtmek için kullanılır (Optional, Dict, List, Tuple)
from typing import Optional, Dict, List, Tuple
# datetime kütüphanesini içeri aktar: Güncel tarih ve saat bilgisini almak için kullanılır
import datetime

# Log ayarları: Programın çalışması sırasında hata ve bilgi mesajlarını tutacak logger nesnesini oluştur
logger = logging.getLogger(__name__)
# logging ayarlarını belirle: INFO seviyesi ve ilerisinde mesajları göster, mesajların başında [LOG_SEVIYESI] yazsın
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

class MavlinkDrone:
    # MavlinkDrone sınıfı: İHA ile iletişim kurmak ve kontrol etmek için gerekli tüm özellikleri ve fonksiyonları içerir
    
    def __init__(self):
        # __init__ fonksiyonu: Nesne oluşturulduğunda çalışır ve tüm özellikleri başlatır
        
        # =====================================================================
        # 1. VEHICLE SINIFI ÖZELLİKLERİ
        # =====================================================================
        # Bu bölüm İHA'nın bağlantısı, durumu, konum bilgisi vb. tüm temel özellikleri tanımlar
        
        # Bağlantı dizesi: İHA'ya hangi port/baud rate ile bağlanacağımızı belirtir (örn: "COM3" veya "/dev/ttyUSB0")
        self.connection_string: str = ""
        # Baud rate: İHA ile haberleşme hızı (57600 bit/saniye standart değer)
        self.baudrate: int = 57600
        # MAVLink bağlantı nesnesi: İHA ile iletişimi sağlayan asıl bağlantı
        self.vehicle: Optional[mavutil.mavlink_connection] = None
        # Master bağlantısı: vehicle nesnesi ile aynıdır, başka kodlarla uyumlu olması için
        self.master = None
        # Bağlanmış mı: True ise İHA'ya başarıyla bağlandığımız anlamına gelir
        self.is_connected: bool = False
        # Son alınan heartbeat mesajı: İHA'nın hayatta olduğunu gösteren düzenli sinyal
        self.last_heartbeat: Optional[Dict] = None
        
        # ========== DURUM BİLGİLERİ ==========
        # Motorlar kilitli mi (False) yoksa açık mı (True): İHA'nın uçabilir durumda olup olmadığını gösterir
        self.is_armed: bool = False
        # Uçuş modu: "MANUAL", "STABILIZE", "GUIDED", "AUTO", "LAND", "RTL" vb. modu gösterir
        self.mode: str = "UNKNOWN"
        # Sistem durumu: "STANDBY" (bekleme), "ACTIVE" (aktif), "CRITICAL" (kritik) vb. sistem durumu
        self.system_status: str = "STANDBY"
        # İHA tipi: "QUADCOPTER", "PLANE", "ROVER" vb. araç türü
        self.vehicle_type: str = ""
        # Firmware versiyonu: İHA'daki yazılımın sürümü (örn: ArduCopter 4.3.0)
        self.firmware_version: str = "Unknown"
        
        # ========== KOORDİNAT VE KONUM ==========
        # Konum sözlüğü: İHA'nın Dünya üzerindeki konumunu derece ve yükseklik cinsinden tutar
        # lat (latitude): Enlem (-90 ile 90 derece arasında)
        # lon (longitude): Boylam (-180 ile 180 derece arasında)
        # alt (altitude): Yer seviyesine göre yükseklik (metre cinsinden)
        self.position: Dict[str, float] = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        
        # Hız sözlüğü: İHA'nın NED (North-East-Down) koordinat sisteminde hızı
        # vx: Kuzey yönü hızı (m/s)
        # vy: Doğu yönü hızı (m/s)
        # vz: Aşağı yönü hızı (m/s) - negatif değer yukarı hareket anlamına gelir
        self.velocity: Dict[str, float] = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        
        # Tutum sözlüğü: İHA'nın X-Y-Z eksenleri etrafındaki dönüş açıları (radyan cinsinden)
        # roll: X ekseni etrafında dönüş (kanat eğmesi)
        # pitch: Y ekseni etrafında dönüş (burun yukarı-aşağı)
        # yaw: Z ekseni etrafında dönüş (yönelim açısı)
        self.attitude: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        
        # ========== ENERJİ (BATARYA) BİLGİSİ ==========
        # Batarya sözlüğü: İHA'nın bataryasının durumu
        # voltage: Batarya voltajı (Volt cinsinden)
        # current: Batarya akımı (Amper cinsinden)
        # level: Batarya seviyesi (0-100 yüzde)
        self.battery: Dict[str, float] = {"voltage": 0.0, "current": 0.0, "level": 0.0}
        
        # ========== GÖREVİ İLİŞKİLİ BİLGİLER ==========
        # Görev listesi: İHA'ya verilecek uçuş noktaları (Waypoint) - her biri (lat, lon, alt) tuple'ı
        self.mission: List[Tuple[float, float, float]] = []
        # Ana konum: İHA'nın kalktığı yer veya belirlenmiş başlangıç noktası
        # Bu noktaya "Return to Launch" (RTL) komutu ile dönebilir
        self.home_location: Dict = {"lat": None, "lon": None, "alt": None}
        
        # ========== ZAMAN VE LOGLAMAEğer İLİŞKİLİ ==========
        # Uçuş başlama zamanı: İHA'nın havalanma süresi (Unix zaman damgası)
        self.flight_start_time: float = 0.0
        # Son mesaj alındığı zaman: En son MAVLink mesajının geldiği zaman
        self.last_message_time: float = 0.0
        # Görünür uydu sayısı: İHA'nın GPS'inin kilitlendiği uydu sayısı (çok yüksek = iyi konum)
        self.satellites_visible: int = 0
        # GPS sabitme tipi: 0 = sabit değil, 1 = 2D sabit, 2 = 3D sabit, 3 = DGPS vs.
        self.gps_fix_type: int = 0
        # Son komut onayı: İHA tarafından işlenen son komutun bilgisi (hata kontrolü için)
        self.last_command_ack: Dict = {}
        # Mesaj tamponu: Son 4 saniyedeki tüm MAVLink mesajlarını burada tutarız
        self.message_buffer: List = []

        # Log sıklığı (saniye): Her kaç saniyede bir loglama yapacağımızı belirtir
        self.log_interval: float = 4.0   # 4 saniyede bir log kaydı oluştur
        # Son log zamanı: En son log kaydı yapıldığında zamanı tutar (çok sık log atmamak için)
        self.last_log_time: float = 0.0
        
        # Telemetri cache (her mesaj türünün son hali):
        # Bu değişkenler son alınan veriyi saklarlar, böylece loglamada hızlı erişebiliriz
        # GPS ham verisi cache'i
        self._cache_gps = None
        # Tutum verisi cache'i
        self._cache_att = None
        # Global konum verisi cache'i
        self._cache_global = None
        # Batarya verisi cache'i
        self._cache_battery = None
        # Heartbeat (kalp atışı) cache'i
        self._cache_heartbeat = None

        # === OTOMATİK TELEMETRİ AYARLARI ===
        # Nesne oluşturulur oluşturulmaz arka planda canlı veri toplayan
        # bir thread başlar. Bağlantı yoksa sadece uyur, bağlantı gelince
        # HEARTBEAT / GPS / GLOBAL_POSITION_INT / ATTITUDE / SYS_STATUS
        # mesajlarını okuyup ilgili alanları günceller.
        
        # Telemetri döngüsü çalışıyor mu: False yaparak telemetri thread'ini durdurabiliz
        self._telemetry_running: bool = True
        # Telemetri hızı: Saniyede kaç kez telemetri verisini okumalıyız (5 = 5 kez/saniye = 200ms aralık)
        self._telemetry_hz: float = 5.0  # saniyede 5 kez
        # Telemetri thread nesnesi: Arka planda çalışacak işlemi temsil eder
        self._telemetry_thread = threading.Thread(
            # _telemetry_loop fonksiyonunu arka planda çalıştır
            target=self._telemetry_loop,
            # daemon=True olması program kapanırken bu thread'in de otomatik kapanması anlamına gelir
            daemon=True
        )
        # Telemetri thread'ini başlat: Nesne oluşturulur oluşturulmaz veri okumaya başlasın
        self._telemetry_thread.start()


    def _telemetry_loop(self):
        """
        Arka planda sürekli MAVLink mesajlarını okuyup
        position / velocity / attitude / battery gibi alanları günceller.
        Nesne oluşturulur oluşturulmaz thread olarak çalışmaya başlar.
        
        Bu fonksiyon sonsuz döngüde çalışır ve:
        1. Bağlantı varsa İHA'dan mesaj okur
        2. Alınan mesajlar türüne göre ilgili özellikleri günceller
        3. Belirlenen hızda (Hz) döngüyü çalıştırır
        """
        # Telemetri döngüsü devam ettiği sürece çalış
        while self._telemetry_running:
            # Döngü başında zamanı kaydederiz (geçen süreyi hesaplamak için)
            start = time.time()

            # Eğer bağlantı yoksa ya da vehicle nesnesi oluşturulmamışsa
            if not (self.vehicle and self.is_connected):
                # 1 saniye uyur ve tekrar kontrol eder (boşu boşuna CPU kullanmasın)
                time.sleep(1.0)
                # Bu iterasyonu atla ve sonrakine geç
                continue

            try:
                # Buffer'daki bütün mesajları hızlıca çek: blocking=False ile hemen döner
                while True:
                    # MAVLink protokolü üzerinden bir mesaj almayı dene (bloklamadan)
                    msg = self.vehicle.recv_match(blocking=False)
                    # Eğer yeni mesaj yoksa döngüyü bitir
                    if not msg:
                        # Hiç mesaj yok, buffer boş
                        break

                    # Alınan mesajın tipini öğren (örn: "HEARTBEAT", "GPS_RAW_INT" vb.)
                    mtype = msg.get_type()
                    
                    # Telemetri cache doldurma (log için son değerleri saklıyoruz)
                    # Böylece loglamada hızlı erişebilir ve güncel verileri kaydedebiliriz
                    
                    # Eğer mesaj GLOBAL_POSITION_INT ise (konum ve hız verisi)
                    if mtype == "GLOBAL_POSITION_INT":
                        # Mesajı sözlük formatına çevir ve cache'e kaydet
                        self._cache_global = msg.to_dict()

                    # Eğer mesaj ATTITUDE ise (tutum/eğim verisi)
                    elif mtype == "ATTITUDE":
                        # Tutum mesajını sözlük formatına çevir ve cache'e kaydet
                        self._cache_att = msg.to_dict()

                    # Eğer mesaj GPS_RAW_INT ise (ham GPS verisi)
                    elif mtype == "GPS_RAW_INT":
                        # GPS mesajını sözlük formatına çevir ve cache'e kaydet
                        self._cache_gps = msg.to_dict()

                    # Eğer mesaj BATTERY_STATUS ise (batarya bilgisi)
                    elif mtype == "BATTERY_STATUS":
                        # Batarya mesajını sözlük formatına çevir ve cache'e kaydet
                        self._cache_battery = msg.to_dict()

                    # Eğer mesaj HEARTBEAT ise (kalp atışı / hayat işareti)
                    elif mtype == "HEARTBEAT":
                        # Heartbeat mesajını sözlük formatına çevir ve cache'e kaydet
                        self._cache_heartbeat = msg.to_dict()


                    # HEARTBEAT mesajı işle: En önemli mesaj, sürekli gelir ve sistem bilgisi verir
                    if mtype == "HEARTBEAT":
                        # Son heartbeat mesajını sakla (geldiğinden emin olmak için)
                        self.last_heartbeat = msg.to_dict()
                        # ARM bilgisini çıkar: Motorların kilitli/açık olup olmadığını belirler
                        try:
                            # base_mode'un belirli biti kontrol edilir (motorlar açık mı sorusu)
                            # MAV_MODE_FLAG_SAFETY_ARMED biti 1 ise motorlar açık demektir
                            self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                        except Exception:
                            # Hata olursa sessizce geç (önemlı değil)
                            pass
                        # Sistem durumunu kaydet: STANDBY, BOOT, CALIBRATING, ARMED vs.
                        self.system_status = str(getattr(msg, "system_status", self.system_status))
                        # UÇUŞ MODU: "MANUAL", "GUIDED", "AUTO", "RTL" vb. modu oku
                        try:
                            # mavutil.mode_string_v10 fonksiyonu mod sayısından mod ismini bulur
                            self.mode = mavutil.mode_string_v10(msg)
                        except Exception:
                            # Mod bulunamadığında sessizce geç
                            pass

                    # GPS ham verisi işle: Coğrafik konum bilgisi
                    elif mtype == "GPS_RAW_INT":
                        # Mesajı sözlük formatına çevir (mesajın tüm alanlarını al)
                        data = msg.to_dict()

                        # GPS'ten gelen lat/lon'u da güncellemek istiyorsan:
                        # Enlem bilgisini al ve 1e7'ye böl (GPS verisinde 7 ondalak basamak olur)
                        self.position["lat"] = data.get("lat", 0) / 1e7
                        # Boylam bilgisini al ve 1e7'ye böl
                        self.position["lon"] = data.get("lon", 0) / 1e7

                        # GPS irtifasını AYRI bir alanda tut (artık alt'ı BOZMAYACAĞIZ)
                        # GPS'ten gelen irtifayı metre cinsinden al (1000'e bölerek)
                        self.position["alt_gps"] = data.get("alt", 0) / 1000.0

                        # Uydu ve fix bilgisi: Konum doğruluğunun ne kadar iyi olduğunu gösterir
                        # Görünür uydu sayısı: Çok yüksek = GPS çalışıyor
                        self.satellites_visible = data.get("satellites_visible", 0)
                        # GPS sabitme tipi: 0 = sabit değil, 2 = 3D sabit (iyi), 3 = DGPS (çok iyi)
                        self.gps_fix_type = data.get("fix_type", 0)

                    # GLOBAL_POSITION_INT mesajı işle: Sistemin hesapladığı kesin konum
                    elif mtype == "GLOBAL_POSITION_INT":
                        # Konum: Latitude (enlem) bilgisini derece cinsine çevir
                        self.position["lat"] = msg.lat / 1e7
                        # Konum: Longitude (boylam) bilgisini derece cinsine çevir
                        self.position["lon"] = msg.lon / 1e7

                        # Deniz seviyesine göre yükseklik (AMSL - Above Mean Sea Level)
                        # Bu mutlak yüksekliktir, coğrafik olarak doğru ama uçuş için kullanışsız
                        self.position["alt_amsl"] = msg.alt / 1000.0

                        # Home (takeoff) noktasına göre yükseklik (RELATIVE ALT)
                        # Bu genelde uçuş için kullanırız çünkü "kaç metre yukarıda" sorusunu cevaplar
                        self.position["alt_rel"] = msg.relative_alt / 1000.0

                        # Varsayılan alt → ARTIK her zaman yerden yükseklik (alt_rel)
                        # Böylece diğer kodlar basitçe position["alt"] kullanabilir
                        self.position["alt"] = self.position["alt_rel"]

                        # Hız vektörü: İHA'nın her yöndeki hızı metre/saniye cinsinden
                        # vx: Kuzey (North) yönü hızı
                        # vy: Doğu (East) yönü hızı
                        # vz: Aşağı (Down) yönü hızı (negatif = yukarı çıkış)
                        self.velocity = {
                            # Vx hızını cm/s'den m/s'ye çevir (100'e bölerek)
                            "vx": msg.vx / 100.0,
                            # Vy hızını cm/s'den m/s'ye çevir
                            "vy": msg.vy / 100.0,
                            # Vz hızını cm/s'den m/s'ye çevir
                            "vz": msg.vz / 100.0,
                        }

                    # ATTITUDE mesajı işle: İHA'nın yönelim (roll, pitch, yaw) bilgisi
                    elif mtype == "ATTITUDE":
                        # Tutum özellikleri güncelleyen (radyan cinsinden)
                        self.attitude = {
                            # Roll: X ekseni etrafında dönüş (kanat eğmesi) - radyan
                            "roll": msg.roll,
                            # Pitch: Y ekseni etrafında dönüş (burun açısı) - radyan
                            "pitch": msg.pitch,
                            # Yaw: Z ekseni etrafında dönüş (yönelim) - radyan
                            "yaw": msg.yaw,
                        }

                    # SYS_STATUS mesajı işle: Sistem durumu (batarya, CPU vb.)
                    elif mtype == "SYS_STATUS":
                        # Batarya bilgisini çıkarmayı dene (bazı sistemlerde bu bilgi olmayabilir)
                        try:
                            # Batarya voltajı: SYS_STATUS mesajında millivolt cinsinden gelir, volta çevir
                            voltage = getattr(msg, "voltage_battery", 0) / 1000.0
                            # Batarya akımı: SYS_STATUS mesajında 0.01A biriminde gelir, ampere çevir
                            current = getattr(msg, "current_battery", 0) / 100.0
                            # Batarya seviyesi: 0-100 yüzde olarak gelir
                            level = getattr(msg, "battery_remaining", 0)
                        except Exception:
                            # Hata olursa tüm değerleri 0 yap
                            voltage, current, level = 0.0, 0.0, 0.0
                        # Batarya sözlüğünü güncelle
                        self.battery = {
                            # Voltaj: Ne kadar güç kaldığını gösterir
                            "voltage": voltage,
                            # Akım: Ne kadar hızlı bittiğini gösterir
                            "current": current,
                            # Seviye: Yüzde cinsinden kalan batarya
                            "level": level,
                        }

                    # Log buffer'ına kaydet (mevcut helper fonksiyonu çağır)
                    self.log_mavlink_message(msg)

            except Exception as e:
                # İstemeden hata oluşursa hata mesajı yazdır ama program devam etsin
                logger.error(f"Telemetri döngüsü hatası: {e}")

            # Hız kontrolü (saniyede ~5 kez çalışması için zamanlamayı ayarla)
            # Geçen zamanı hesapla
            elapsed = time.time() - start
            # İstenilen hıza göre ne kadar uymalıyız hesapla
            # (1 / 5Hz = 0.2 saniye, ama işlem zaman alırsa fark düşülür)
            sleep_time = max(0.0, (1.0 / self._telemetry_hz) - elapsed)
            # Eğer uyku zamanı pozitif ise uyut (döngü çok hızlı olmayan)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_telemetry_snapshot(self) -> Dict[str, Dict]:
        """
        Panel için tek seferde mümkün olduğunca kapsamlı anlık telemetri verisi sağlar.
        
        Bu fonksiyon arayüz (UI) panellerinin güncel bilgi gösterebilmesi için
        tüm telemetri verilerini bir sözlükte toplar ve döndürür.
        
        Parametreler:
            Yok (self kullanır)
        
        Döndürülenler:
            Dict[str, Dict]: Tüm telemetri verilerini içeren sözlük:
                - position: Konum bilgisi (lat, lon, alt)
                - velocity: Hız bilgisi (vx, vy, vz)
                - attitude: Tutum bilgisi (roll, pitch, yaw)
                - battery: Batarya bilgisi (voltage, current, level)
                - is_armed: Motorlar açık mı?
                - mode: Uçuş modu
                - system_status: Sistem durumu
                - satellites_visible: Görünür uydu sayısı
                - gps_fix_type: GPS sabitme kalitesi
                - is_connected: Bağlı mı?
                - connection_string: Bağlantı dizesi
                - last_heartbeat: Son heartbeat mesajı
                - message_buffer_len: Buffer'daki mesaj sayısı
                - last_message_time: Son mesaj zamanı
        """
        return {
            # Konum / hız / tutum bilgisi: Uçuşun mühendislik parametreleri
            # position'u kopyala (.copy() ile yeni sözlük oluştur, orijinal değiştirilmesin)
            "position": self.position.copy(),
            # velocity'yi kopyala (kuzey, doğu, aşağı hızı)
            "velocity": self.velocity.copy(),
            # attitude'ı kopyala (roll, pitch, yaw açıları)
            "attitude": self.attitude.copy(),

            # Enerji bilgisi: İHA'nın ne kadar bataryası kaldığı
            # battery'yi kopyala (voltaj, akım, seviye)
            "battery": self.battery.copy(),

            # Durum bilgileri: İHA'nın genel durumu ve sistem sağlığı
            # Motorlar kilitli mi açık mı (True = açık, False = kilitli)
            "is_armed": self.is_armed,
            # Uçuş modu: "MANUAL", "GUIDED", "AUTO", "RTL" vb.
            "mode": self.mode,
            # Sistem durumu: "STANDBY", "ACTIVE", "CRITICAL" vb.
            "system_status": self.system_status,
            # Görünür uydu sayısı: Ne kadar iyi GPS kilidi var
            "satellites_visible": self.satellites_visible,
            # GPS sabitme tipi: 0 = sabit değil, 2 = 3D, 3 = DGPS
            "gps_fix_type": self.gps_fix_type,

            # Bağlantı ve ham veriler: İHA ile bağlantı durumu
            # İHA'ya bağlı mı (True = bağlı, False = kopuk)
            "is_connected": self.is_connected,
            # Hangi port/hız ile bağlı olduğu (COM3, /dev/ttyUSB0 vb.)
            "connection_string": self.connection_string,
            # En son alınan heartbeat mesajı (sistemin sağlığını gösterir)
            "last_heartbeat": self.last_heartbeat,
            # Mesaj buffer'ında kaç adet mesaj vardır (log verisi)
            "message_buffer_len": len(self.message_buffer),
            # En son mesaj alınma zamanı
            "last_message_time": self.last_message_time,
        }

    def stop_telemetry(self):
        """
        Telemetri thread'ini temiz şekilde durdurur.
        
        Program kapatılırken veya bağlantı kesileceğinde bu fonksiyon çağrılır.
        Bu sayede arka planda çalışan thread düzgün şekilde kapanır.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            None
        """
        # Telemetri döngüsü bayrağını False yap, böylece _telemetry_loop while döngüsü bitsin
        self._telemetry_running = False

    # =========================================================================
    # 2. FONKSİYON LİSTESİ
    # =========================================================================

    # --- BAĞLANTI GRUBU ---

    def connect(self, connection_string: str, baudrate: int = 57600, timeout: int = 10):
        """
        İHA'ya MAVLink protokolü üzerinden bağlantı kurar.
        
        Bu fonksiyon İHA ile iletişim kurmak için serial port veya network bağlantısını açar.
        Bağlantı açıldığında heartbeat (kalp atışı) mesajları alana kadar bekler.
        
        Parametreler:
            connection_string (str): Bağlantı adresi
                - Seri port: "COM3" (Windows) veya "/dev/ttyUSB0" (Linux)
                - Network: "192.168.1.1:14550" (UDP ile)
            baudrate (int): İletişim hızı (saniyede kaç bit)
                - Varsayılan: 57600 (standart İHA baud rate)
            timeout (int): Kaç saniye heartbeat bekleyeceğiz (varsayılan: 10)
        
        Döndürülenler:
            bool: True (başarılı bağlantı), False (bağlantı başarısız)
        """
        # Bağlantı dizesini sakla (debug ve bilgi için)
        self.connection_string = connection_string
        # Baud rate'i sakla (debug için)
        self.baudrate = baudrate
        # Bağlantı kuruluyor mesajı yazdır
        logger.info(f"Bağlantı kuruluyor: {connection_string}")
        try:
            # MAVLink bağlantısını kur: pymavlib ile serial/network açık
            self.vehicle = mavutil.mavlink_connection(connection_string, baud=baudrate)
            # master da aynı bağlantıya işaret et (uyumluluk için)
            self.master = self.vehicle
            # Şu andaki zamanı kaydediriz (zaman aşımı kontrolü için)
            start = time.time()
            # Timeout (zaman sınırı) bitene kadar HEARTBEAT mesajı bekle
            while time.time() - start < timeout:
                # HEARTBEAT mesajını timeout kadar bekle (maksimum 1 saniye bekle)
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                # Eğer heartbeat mesajı alındı ise
                if msg:
                    # Heartbeat mesajını sözlük olarak sakla
                    self.last_heartbeat = msg.to_dict()
                    # Bağlı olduğumuzu işaretle
                    self.is_connected = True
                    # Uçuş modunu oku ve kaydet
                    try:
                        # mavutil.mode_string_v10 fonksiyonu mod ID'sinden mod ismine çevir
                        self.mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        # Mod bulunamazsa sessizce devam et
                        pass
                    # Başarıyla bağlandığımızı logla
                    logger.info(f"Bağlantı kuruldu, mod: {self.mode}")
                    # Fonksiyondan başarı döndür
                    return True

            # Eğer timeout bitene kadar heartbeat alamadıysak
            logger.error("ZAMAN AŞIMI: Heartbeat alınamadı.")
            # Bağlı olmadığımızı işaretle
            self.is_connected = False
            # Başarısızlığı döndür
            return False
        except Exception as e:
            # Bağlantı sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Bağlantı hatası: {e}")
            # Bağlı olmadığımızı işaretle
            self.is_connected = False
            # Başarısızlığı döndür
            return False

    def wait_heartbeat(self, timeout: int = 10):
        """
        İHA'dan gelen heartbeat (kalp atışı) mesajını belirtilen süre kadar bekler.
        
        Bağlantı kurulduğundan emin olmak veya bağlantının hala aktif olup olmadığını
        kontrol etmek için kullanılır. Heartbeat mesajı İHA tarafından saniyede bir gönderilir.
        
        Parametreler:
            timeout (int): Kaç saniye heartbeat bekleyeceğiz (varsayılan: 10)
        
        Döndürülenler:
            Dict veya None: Heartbeat mesajı başarıyla alındıysa mesaj sözlüğü döndür,
                            aksi halde None döndür
        """
        # Eğer bağlantı nesnesi oluşturulmamışsa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Önce connect() çağrılmalı.")
            # Hiçbir şey döndürme (None)
            return None
        # Heartbeat bekleniyor mesajı yazdır
        logger.info("Heartbeat bekleniyor...")
        try:
            # Heartbeat mesajını timeout kadar bekle
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
            # Eğer heartbeat mesajı alındıysa
            if msg:
                # Heartbeat mesajını sözlük olarak sakla
                self.last_heartbeat = msg.to_dict()
                # Bağlı olduğumuzu işaretle
                self.is_connected = True
                # Uçuş modunu oku ve kaydet
                try:
                    # Mod ID'sini mod isminden oku
                    self.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    # Mod bulunamazsa sessizce devam et
                    pass
                # Başarıyla heartbeat aldığımızı logla
                logger.info(f"Heartbeat alındı, mod: {self.mode}")
                # Heartbeat mesajını döndür
                return self.last_heartbeat

            # Timeout bitene kadar heartbeat alinamadıysa hata yazdır
            logger.error("Heartbeat alınamadı (zaman aşımı).")
            # Başarısızlığı işaretle (None döndür)
            return None
        except Exception as e:
            # Heartbeat alırken hata oluşursa hata mesajı yazdır
            logger.error(f"Heartbeat hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    def get_last_heartbeat(self):
        """
        Son alınan heartbeat mesajını döndürür.
        
        İHA'nın durumunu kontrol etmek için kullanılır. None döndürürse
        hiç heartbeat alınmamış demektir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Son alınan heartbeat mesajı veya None
        """
        # Son alınan heartbeat mesajını döndür
        return self.last_heartbeat

    # --- TELEMETRİ OKUMA GRUBU ---

    def get_gps(self):
        """
        İHA'nın GPS konumunu okur (latitude, longitude, altitude).
        
        Bu fonksiyon GPS_RAW_INT mesajını okur ve konum bilgisini günceller.
        Ayrıca uydu sayısı ve GPS sabitme kalitesi hakkında da bilgi verir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Konum bilgileri içeren sözlük veya hata durumunda None
                {
                    "lat": Enlem (derece),
                    "lon": Boylam (derece),
                    "alt": Irtifa (metre),
                    "satellites": Görünür uydu sayısı,
                    "fix_type": Sabitme kalitesi (0=hiçbiri, 2=3D, 3=DGPS)
                }
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("GPS verisi için bağlantı yok.")
            # Başarısızlığı işaretle (None döndür)
            return None
        try:
            # GPS_RAW_INT mesajını timeout=1 ile bekle
            msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            # Eğer GPS mesajı alınamadıysa
            if not msg:
                # Uyarı mesajı yazdır
                logger.warning("GPS verisi alınamadı.")
                # Başarısızlığı işaretle (None döndür)
                return None
            # Mesajı sözlüğe çevir (tüm alanları erişilebilir hale getir)
            data = msg.to_dict()
            # Enlem bilgisini al ve 1e7'ye böl (GPS verisi 7 ondalak basamakta gelir)
            self.position["lat"] = data.get("lat", 0) / 1e7
            # Boylam bilgisini al ve 1e7'ye böl
            self.position["lon"] = data.get("lon", 0) / 1e7
            # Irtifa bilgisini al ve 1000'e böl (mm'den metre'ye çevir)
            self.position["alt"] = data.get("alt", 0) / 1000
            # Görünür uydu sayısını al (çok = iyi)
            self.satellites_visible = data.get("satellites_visible", 0)
            # GPS sabitme türünü al
            self.gps_fix_type = data.get("fix_type", 0)
            # Toplanan konum bilgisini sözlük olarak döndür
            return {
                # Enlem (derece)
                "lat": self.position["lat"],
                # Boylam (derece)
                "lon": self.position["lon"],
                # Irtifa (metre)
                "alt": self.position["alt"],
                # Görünür uydu sayısı
                "satellites": self.satellites_visible,
                # Sabitme türü
                "fix_type": self.gps_fix_type
            }
        except Exception as e:
            # GPS okuma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"GPS hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    def get_attitude(self):
        """
        İHA'nın tutumunu (roll, pitch, yaw) okur - bu değerler radyan cinsindedir.
        
        Tutum bilgisi İHA'nın X, Y, Z eksenleri etrafında döndüğü açıları gösterir.
        Bu sayede İHA'nın eğim ve yönelim bilgisi alınabilir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Tutum bilgileri (roll, pitch, yaw radyan cinsinden)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Tutum verisi için bağlantı yok.")
            # Başarısızlığı işaretle (None döndür)
            return None
        try:
            # ATTITUDE mesajını timeout=1 ile bekle
            msg = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            # Eğer ATTITUDE mesajı alınamadıysa
            if not msg:
                # Uyarı mesajı yazdır
                logger.warning("ATTITUDE mesajı alınamadı.")
                # Başarısızlığı işaretle (None döndür)
                return None
            # Roll açısını güncelle (X ekseni etrafında dönüş) - radyan
            self.attitude["roll"] = msg.roll
            # Pitch açısını güncelle (Y ekseni etrafında dönüş) - radyan
            self.attitude["pitch"] = msg.pitch
            # Yaw açısını güncelle (Z ekseni etrafında dönüş / yönelim) - radyan
            self.attitude["yaw"] = msg.yaw
            # Tutum bilgisini kopyalayarak döndür (orijinal değiştirilmesin)
            return self.attitude.copy()
        except Exception as e:
            # Tutum okuma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Attitude hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    def get_global_position(self):
        """
        İHA'nın global konumunu okur (GLOBAL_POSITION_INT mesajından).
        
        Bu konumu, İHA tarafından havada hesaplanan ve GPS+IMU verilerinden
        oluşan en doğru konum değeridir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Konum bilgileri (lat, lon, alt)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Global konum için bağlantı yok.")
            # Başarısızlığı işaretle (None döndür)
            return None
        try:
            # GLOBAL_POSITION_INT mesajını timeout=1 ile bekle
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            # Eğer global konum mesajı alınamadıysa
            if not msg:
                # Uyarı mesajı yazdır
                logger.warning("GLOBAL_POSITION_INT alınamadı.")
                # Başarısızlığı işaretle (None döndür)
                return None
            # Enlem bilgisini al ve 1e7'ye böl (derece cinsine çevir)
            self.position["lat"] = msg.lat / 1e7
            # Boylam bilgisini al ve 1e7'ye böl (derece cinsine çevir)
            self.position["lon"] = msg.lon / 1e7
            # Irtifa bilgisini al ve 1000'e böl (mm'den metre'ye çevir)
            self.position["alt"] = msg.alt / 1000.0
            # Konum bilgisini kopyalayarak döndür (orijinal değiştirilmesin)
            return self.position.copy()
        except Exception as e:
            # Global konum okuma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Global konum hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    def get_velocity(self):
        """
        İHA'nın hızını NED (North-East-Down) koordinat sisteminde okur.
        
        NED koordinat sistemi:
        - Kuzey (North) = pozitif X yönü
        - Doğu (East) = pozitif Y yönü
        - Aşağı (Down) = pozitif Z yönü
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Hız bilgileri (vx, vy, vz) m/s cinsinden
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Hız verisi için bağlantı yok.")
            # Başarısızlığı işaretle (None döndür)
            return None
        try:
            # GLOBAL_POSITION_INT mesajını timeout=1 ile bekle (hız verisi bu mesajda gelir)
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            # Eğer hız mesajı alınamadıysa
            if not msg:
                # Uyarı mesajı yazdır
                logger.warning("GLOBAL_POSITION_INT hız için alınamadı.")
                # Başarısızlığı işaretle (None döndür)
                return None
            # Kuzey yönü hızını al ve 100'e böl (cm/s'den m/s'ye çevir)
            self.velocity["vx"] = msg.vx / 100.0
            # Doğu yönü hızını al ve 100'e böl (cm/s'den m/s'ye çevir)
            self.velocity["vy"] = msg.vy / 100.0
            # Aşağı yönü hızını al ve 100'e böl (cm/s'den m/s'ye çevir)
            # Negatif değer yukarı çıkış anlamına gelir
            self.velocity["vz"] = msg.vz / 100.0
            # Hız bilgisini kopyalayarak döndür (orijinal değiştirilmesin)
            return self.velocity.copy()
        except Exception as e:
            # Hız okuma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Hız hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    def get_battery(self):
        """
        İHA'nın bataryası hakkında bilgi okur (voltaj, akım, seviye).
        
        Batarya bilgisi, İHA'nın ne kadar uzun süre daha uçabileceğini
        belirlemek için çok önemlidir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            Dict veya None: Batarya bilgileri (voltage, current, level)
                - voltage: Voltaj (Volt)
                - current: Akım (Amper)
                - level: Seviye (0-100 yüzde)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Batarya verisi için bağlantı yok.")
            # Başarısızlığı işaretle (None döndür)
            return None
        try:
            # SYS_STATUS mesajını timeout=1 ile bekle (batarya bilgisi bu mesajda gelir)
            msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            # Eğer SYS_STATUS mesajı alınamadıysa
            if not msg:
                # Uyarı mesajı yazdır
                logger.warning("SYS_STATUS mesajı alınamadı.")
                # Başarısızlığı işaretle (None döndür)
                return None
            # Batarya voltajını al ve 1000'e böl (mV'dan V'a çevir)
            self.battery["voltage"] = msg.voltage_battery / 1000.0
            # Batarya akımını al ve 100'e böl (0.01A biriminden A'a çevir)
            self.battery["current"] = msg.current_battery / 100.0
            # Batarya seviyesini al (0-100 yüzde olarak gelir)
            self.battery["level"] = msg.battery_remaining
            # Batarya bilgisini kopyalayarak döndür (orijinal değiştirilmesin)
            return self.battery.copy()
        except Exception as e:
            # Batarya okuma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Batarya hatası: {e}")
            # Başarısızlığı işaretle (None döndür)
            return None

    # --- MOD ve KONTROL KOMUTLARI ---

    def set_flight_mode(self, mode: str):
        """
        İHA'nın uçuş modunu değiştirir.
        
        Mevcut modlar: MANUAL, STABILIZE, ALT_HOLD, LOITER, GUIDED, AUTO, RTL, LAND vb.
        Her mod farklı özellikleri etkinleştirir (örn: GUIDED = kontrol altında komut alma).
        
        Parametreler:
            mode (str): Değiştirmek istediğimiz mod (örn: "GUIDED", "RTL", "LAND")
        
        Döndürülenler:
            bool: True (başarılı), False (başarısız)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Önce connect() çağrılmalı.")
            # Başarısızlığı işaretle (False döndür)
            return False
        # Mod ismini büyük harfe çevir (örn: "guided" → "GUIDED")
        mode = mode.upper()
        try:
            # Eğer istediğimiz mod, İHA'nın desteklediği modlar içinde yoksa
            if mode not in self.vehicle.mode_mapping():
                # Hata mesajı yazdır (desteklenmeyen mod)
                logger.error(f"Geçersiz mod: {mode}")
                # Başarısızlığı işaretle (False döndür)
                return False
            # Mod ismini ID numarasına çevir (İHA ile iletişim için sayı kullanılır)
            mode_id = self.vehicle.mode_mapping()[mode]
            # Mod değiştirme komutunu gönder
            # target_system: Hedef sistem ID'si (İHA'nın ID'si, varsayılan 1)
            # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: Özel modlar etkin
            # mode_id: Değiştirmek istediğimiz modun ID'si
            self.vehicle.mav.set_mode_send(
                # Hedef sistem ID'si (genelde 1)
                self.vehicle.target_system,
                # MAV mod bayrağı (özel modlar kullan)
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                # Hedef mod ID'si
                mode_id
            )
            # Lokal olarak modu güncelle (hemen geçsin diye)
            self.mode = mode
            # Başarıyı logla
            logger.info(f"Mod değiştirildi: {mode}")
            # Başarıyı işaretle (True döndür)
            return True
        except Exception as e:
            # Mod değiştirme sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Mod değiştirme hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    def get_flight_mode(self):
        """
        Şu anki uçuş modunu döndürür.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            str: Şu anki uçuş modu (örn: "GUIDED", "RTL", "LAND")
        """
        # Lokal olarak saklanan modu döndür
        return self.mode

    def goto_gps_location(self, lat: float, lon: float, alt: float):
        """
        İHA'yı belirlenmiş GPS konumuna gönderir.
        
        Bu fonksiyon, GUIDED modda kullanılır. İHA'ya belirli bir coğrafik
        konuma ve irtifaya gitmesini söyler.
        
        Parametreler:
            lat (float): Hedef enlem (derece, örn: 40.5)
            lon (float): Hedef boylam (derece, örn: 29.8)
            alt (float): Hedef irtifa (metre, home noktasına göre)
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("goto_gps_location için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # Hedef konumu logla (debug için)
            logger.info(f"Yeni konuma gidiliyor: lat={lat}, lon={lon}, alt={alt}")
            # Mod'u GUIDED'a çevir (İHA kontrol komutlarını kabul etmek için)
            self.set_flight_mode("GUIDED")
            # SET_POSITION_TARGET_GLOBAL_INT komutu gönder (kesin konum hedefi)
            # Bu komut İHA'ya "bu enlem, boylam, irtifaya git" der
            self.vehicle.mav.set_position_target_global_int_send(
                # Zaman damgası (0 = şu an)
                0,
                # Hedef sistem ID (genelde 1)
                self.vehicle.target_system,
                # Hedef bileşen ID (genelde 1)
                self.vehicle.target_component,
                # Koordinat sistemi (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = home'a göre irtifa)
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                # Tip maskesi (hangi parametrelerin geçerli olduğu)
                # 0b110111111000 = sadece konum hedefi (hız, ivme vb. yok)
                0b110111111000,
                # Hedef enlem (1e7 ile çarpılmış format: 40.5 → 405000000)
                int(lat * 1e7),
                # Hedef boylam (1e7 ile çarpılmış format)
                int(lon * 1e7),
                # Hedef irtifa (metre, home noktasına göre)
                alt,
                # Hız X (m/s) - kullanılmıyor (maske yüzünden)
                0,
                # Hız Y (m/s) - kullanılmıyor
                0,
                # Hız Z (m/s) - kullanılmıyor
                0,
                # İvme X - kullanılmıyor
                0,
                # İvme Y - kullanılmıyor
                0,
                # İvme Z - kullanılmıyor
                0,
                # Yaw (yönelim) - kullanılmıyor
                0,
                # Yaw hızı - kullanılmıyor
                0
            )
            # Başarıyı işaretle (True döndür)
            return True
        except Exception as e:
            # Goto komutu gönderme sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Goto hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    def set_velocity(self, vx: float, vy: float, vz: float):
        """
        İHA'ya hız vektörü verir (NED koordinat sisteminde).
        
        Bu fonksiyon, İHA'ya belirli bir yönde ve hızda hareket etmesini söyler.
        Genelde manuel kontrol veya velocity-based görevler için kullanılır.
        
        Parametreler:
            vx (float): Kuzey yönü hızı (m/s, pozitif = kuzey)
            vy (float): Doğu yönü hızı (m/s, pozitif = doğu)
            vz (float): Aşağı yönü hızı (m/s, pozitif = aşağı, negatif = yukarı)
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Hız komutu için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # SET_POSITION_TARGET_LOCAL_NED komutu gönder (NED hız hedefi)
            self.vehicle.mav.set_position_target_local_ned_send(
                # Zaman damgası (0 = şu an)
                0,
                # Hedef sistem ID
                self.vehicle.target_system,
                # Hedef bileşen ID
                self.vehicle.target_component,
                # Koordinat sistemi (MAV_FRAME_LOCAL_NED = yerel NED)
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                # Tip maskesi (sadece hız geçerli)
                # 0b0000111111000111 = sadece hız alanları
                0b0000111111000111,
                # Hedef X konumu - kullanılmıyor
                0,
                # Hedef Y konumu - kullanılmıyor
                0,
                # Hedef Z konumu - kullanılmıyor
                0,
                # Hedef Vx hızı (kuzey, m/s)
                vx,
                # Hedef Vy hızı (doğu, m/s)
                vy,
                # Hedef Vz hızı (aşağı, m/s)
                vz,
                # İvme X - kullanılmıyor
                0,
                # İvme Y - kullanılmıyor
                0,
                # İvme Z - kullanılmıyor
                0,
                # Yaw - kullanılmıyor
                0,
                # Yaw hızı - kullanılmıyor
                0
            )
            # Komut gönderilme başarısını logla
            logger.info(f"Hız komutu gönderildi: vx={vx}, vy={vy}, vz={vz}")
            # Başarıyı işaretle (True döndür)
            return True
        except Exception as e:
            # Hız komutu gönderme sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Hız komutu hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    def set_attitude(self, roll: float, pitch: float, yaw: float, thrust: float=0.5):
        """
        İHA'nın tutumunu (roll, pitch, yaw) ayarlar.
        
        Bu düşük seviyeli komuttur - İHA'ya doğrudan ne kadar eğilmesi gerektiğini söyler.
        Genelde otopilot tarafından kullanılır.
        
        Parametreler:
            roll (float): Roll açısı (X ekseni, radyan)
            pitch (float): Pitch açısı (Y ekseni, radyan)
            yaw (float): Yaw açısı (Z ekseni, radyan)
            thrust (float): Itki seviyesi (0.0 - 1.0, varsayılan 0.5)
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Attitude komutu için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # SET_ATTITUDE_TARGET komutu gönder
            self.vehicle.mav.set_attitude_target_send(
                # Zaman damgası (0 = şu an)
                0,
                # Hedef sistem ID
                self.vehicle.target_system,
                # Hedef bileşen ID
                self.vehicle.target_component,
                # Tip maskesi (rotation quaternion kullan)
                # 0b00000111 = roll, pitch, yaw hedefi
                0b00000111,
                # Quaternion (roll, pitch, yaw'dan dönüştürülmüş)
                # _to_quaternion fonksiyonu kullanılır
                self._to_quaternion(roll, pitch, yaw),
                # Angular velocity X (rad/s) - kullanılmıyor
                0,
                # Angular velocity Y (rad/s) - kullanılmıyor
                0,
                # Angular velocity Z (rad/s) - kullanılmıyor
                0,
                # Thrust (itki): 0.0 - 1.0 arası değer
                thrust
            )
            # Komut gönderilme başarısını logla
            logger.info(f"Attitude komutu gönderildi: roll={roll}, pitch={pitch}, yaw={yaw}")
            # Başarıyı işaretle (True döndür)
            return True
        except Exception as e:
            # Attitude komutu gönderme sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Attitude komutu hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    def send_ned_velocity(self, vx: float, vy: float, vz: float, duration: float = 1.0):
        """
        İHA'ya NED hızı verir ve belirli süre boyunca tekrar tekrar gönderir.
        
        set_velocity() fonksiyonundan farkı: Bu fonksiyon belirli bir süre boyunca
        hız komutunu tekrar tekrar gönderir. Bağlantı kesintisine karşı dayanıklıdır.
        
        Parametreler:
            vx (float): Kuzey yönü hızı (m/s)
            vy (float): Doğu yönü hızı (m/s)
            vz (float): Aşağı yönü hızı (m/s)
            duration (float): Kaç saniye boyunca bu hızı koruyacak (varsayılan: 1.0)
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("send_ned_velocity için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # Bitiş zamanını hesapla (şu an + duration saniye)
            end_time = time.time() + duration
            # Bitiş zamanına kadar döngü yap (komutu tekrar tekrar gönder)
            while time.time() < end_time:
                # Hız komutunu gönder (set_position_target_local_ned)
                self.vehicle.mav.set_position_target_local_ned_send(
                    # Zaman damgası
                    0,
                    # Hedef sistem ID
                    self.vehicle.target_system,
                    # Hedef bileşen ID
                    self.vehicle.target_component,
                    # Koordinat sistemi (NED)
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    # Tip maskesi (sadece hız)
                    0b0000111111000111,
                    # Konum X - kullanılmıyor
                    0,
                    # Konum Y - kullanılmıyor
                    0,
                    # Konum Z - kullanılmıyor
                    0,
                    # Hız Vx (kuzey)
                    vx,
                    # Hız Vy (doğu)
                    vy,
                    # Hız Vz (aşağı)
                    vz,
                    # İvme X - kullanılmıyor
                    0,
                    # İvme Y - kullanılmıyor
                    0,
                    # İvme Z - kullanılmıyor
                    0,
                    # Yaw - kullanılmıyor
                    0,
                    # Yaw hızı - kullanılmıyor
                    0
                )
                # 100ms (0.1 saniye) bekle (çok sık komut göndermemek için)
                time.sleep(0.1)
            # Başarıyı logla
            logger.info(f"NED velocity {duration}s boyunca uygulandı.")
            # Başarıyı işaretle (True döndür)
            return True
        except Exception as e:
            # send_ned_velocity sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"send_ned_velocity hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 15. load_mission
    def load_mission(self, mission_items: List[Tuple[float, float, float]]):
        """
        İHA'ya görev listesini yükler (bellek de kaydeder, İHA'ya göndermez).
        
        Bu fonksiyon sadece yerel belleğe yükler. Gerçek görev kaydı için
        start_mission() fonksiyonu çağrılmalıdır.
        
        Parametreler:
            mission_items (List[Tuple[float, float, float]]): Görev noktaları listesi
                Her öğe: (enlem, boylam, irtifa) tuple'ı
                Örn: [(40.5, 29.8, 30), (40.51, 29.81, 50), (40.52, 29.82, 30)]
        
        Döndürülenler:
            bool: Daima True (basit yükleme işlemi)
        """
        # Görev listesini sakla (self.mission'a ata)
        self.mission = mission_items
        # Kaç adet görev noktası yüklendiğini logla
        logger.info(f"Görev yüklendi: {len(mission_items)} adet WP")
        # Başarı döndür (daima başarılı)
        return True

    # 16. start_mission
    def start_mission(self):
        """
        Yüklü olan görev listesini başlatır.
        
        Bu fonksiyon tüm görev noktalarında sırayla durdur (halt) yaparak
        İHA'yı her noktaya götürür. Her noktaya vardığında biraz bekler.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: True (görev başlatıldı), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Görev başlatmak için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        # Eğer görev listesi boşsa
        if not self.mission:
            # Hata mesajı yazdır
            logger.error("Görev listesi boş.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # Görev başlatılıyor mesajı yazdır
            logger.info("Görev başlatılıyor...")
            # Basit örnek: GUIDED modda sırayla goto_gps_location çağırma
            # mission_items liste: her biri (lat, lon, alt) tuple
            for (lat, lon, alt) in self.mission:
                # Şu anki görev noktasına git (GUIDED modda)
                self.goto_gps_location(lat, lon, alt)
                # Sonraki noktaya geçmeden önce 2 saniye bekle (İHA'nın hareket etmesi için)
                time.sleep(2)
            # Görev başarıyla tamamlandı
            return True
        except Exception as e:
            # Görev başlatma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"start_mission hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 17. pause_mission (basitleştirilmiş)
    def pause_mission(self):
        """
        Görevini duraklatır (basitleştirilmiş sürüm).
        
        ÖNEMLİ: Bu sürüm sadece log kaydı yapar, gerçek hold uygulamaz.
        Gerçek durma için set_flight_mode("LOITER") kullanılabilir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: Daima True
        """
        # Görev duraklatıldığı loglanır
        logger.info("Görev duraklatıldı (sadece log; gerçek hold uygulanmadı).")
        # Daima True döndür (basit log fonksiyonu)
        return True

    # 18. clear_mission
    def clear_mission(self):
        """
        Yüklü olan görev listesini temizler.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: Daima True
        """
        # Görev listesini boşalt
        self.mission.clear()
        # Görev temizlendi mesajı yazdır
        logger.info("Görev temizlendi.")
        # Başarı döndür
        return True

    # 19. set_rc_channel_pwm
    def set_rc_channel_pwm(self, channel: int, pwm: int):
        """
        RC kanalını PWM değeri ile geçersiz kılar (override).
        
        Bu fonksiyon, kumanda istasyonunun sinyalini geçersiz kılıp,
        program tarafından kontrol etmek için kullanılır. İHA hazırlama
        ve test sırasında yararlı olabilir.
        
        Parametreler:
            channel (int): RC kanalı numarası (1-18)
                1 = Roll, 2 = Pitch, 3 = Throttle, 4 = Yaw
            pwm (int): PWM değeri (1000 - 2000 arası)
                1000 = minimum, 1500 = orta, 2000 = maksimum
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("RC override için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # 18 kanallık PWM değer dizisi oluştur (tüm kanallar devre dışı = 65535)
            rc_channel_values = [65535] * 18
            # İstediğimiz kanalın değerini PWM değerine ayarla (0-tabanlı indeks)
            rc_channel_values[channel - 1] = pwm
            # RC override komutunu gönder (tüm kanallarla birlikte)
            self.vehicle.mav.rc_channels_override_send(
                # Hedef sistem ID
                self.vehicle.target_system,
                # Hedef bileşen ID
                self.vehicle.target_component,
                # 18 kanallık PWM değerleri (dizinin açılımı)
                *rc_channel_values
            )
            # Override başarıyı logla
            logger.info(f"RC channel {channel} override: {pwm}")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # RC override sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"RC override hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 20. arm
    def arm(self):
        """
        İHA'nın motorlarını ARM eder (kilitli durumdan açık duruma çevirir).
        
        ARM etmek, İHA'nın propellerlerini döndürmeye hazır olduğu anlamına gelir.
        Havalanmadan önce ARM etmek gerekir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: True (ARM komutu gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("ARM için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # ArduCopter/Copter motorlarını ARM et
            self.vehicle.arducopter_arm()
            # Lokal olarak ARM durumunu işaretle
            self.is_armed = True
            # ARM başarısını logla
            logger.info("Motorlar ARM edildi.")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # ARM sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"ARM hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 21. disarm
    def disarm(self):
        """
        İHA'nın motorlarını DISARM eder (açık durumdan kilitli duruma çevirir).
        
        DISARM etmek, İHA'nın propellerlerini durdurduğu anlamına gelir.
        Uçuş bittikten sonra güvenlik için DISARM etmek gerekir.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: True (DISARM komutu gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("DISARM için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # ArduCopter/Copter motorlarını DISARM et
            self.vehicle.arducopter_disarm()
            # Lokal olarak DISARM durumunu işaretle
            self.is_armed = False
            # DISARM başarısını logla
            logger.info("Motorlar DISARM edildi.")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # DISARM sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"DISARM hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 22. rtl
    def rtl(self):
        """
        İHA'yı "Return To Launch" (Başlangıç Noktasına Dön) moduna alır.
        
        Bu mod, İHA'yı kalktığı yere otomatik olarak geri getirir.
        Kaybolma veya acil durumlarda kullanılır.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: True (mod değiştirildi), False (hata)
        """
        # Uçuş modunu RTL'ye değiştir (set_flight_mode fonksiyonunu kullan)
        return self.set_flight_mode("RTL")

    # 23. land
    def land(self):
        """
        İHA'yı iniş moduna alır.
        
        Bu mod, İHA'yı mevcut konum üzerinde indirmeye başlar.
        Kontrollü inişi sağlar.
        
        Parametreler:
            Yok
        
        Döndürülenler:
            bool: True (mod değiştirildi), False (hata)
        """
        # Uçuş modunu LAND'a değiştir (set_flight_mode fonksiyonunu kullan)
        return self.set_flight_mode("LAND")

    # 24. takeoff
    def takeoff(self, altitude: float):
        """
        İHA'yı belirtilen irtifaya havalandırır.
        
        Bu fonksiyon, İHA'yı otomatik olarak kalkış yaptırır.
        Belirtilen irtifaya ulaştığında kalması beklenir.
        
        Parametreler:
            altitude (float): Kalkış irtifası (metre, home noktasından göreceli)
        
        Döndürülenler:
            bool: True (kalkış komutu gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("Kalkış için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # Mod'u GUIDED'a değiştir (kalkış komutunu kabul etmek için)
            self.set_flight_mode("GUIDED")
            # Kalkış komutunu gönder (MAV_CMD_NAV_TAKEOFF)
            self.vehicle.mav.command_long_send(
                # Hedef sistem ID
                self.vehicle.target_system,
                # Hedef bileşen ID
                self.vehicle.target_component,
                # Komut: Kalkış
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                # Confirmation (başarı onayı) - 0 = ilk deneme
                0,
                # Param 1: Pitch açısı (0 = varsayılan) - kullanılmıyor
                0,
                # Param 2: Boş
                0,
                # Param 3: Boş
                0,
                # Param 4: Boş
                0,
                # Param 5: X (boş)
                0,
                # Param 6: Y (boş)
                0,
                # Param 7: Hedef irtifa (metre)
                altitude
            )
            # Kalkış komutunu logla
            logger.info(f"{altitude}m irtifaya kalkış komutu gönderildi.")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # Kalkış sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Takeoff hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 25. change_altitude
    def change_altitude(self, altitude: float):
        """
        İHA'nın mevcut konumda irtifasını değiştirir.
        
        Bu fonksiyon, İHA'nın enlem/boylam'ını sabit tutup, sadece
        irtifasını değiştirir. İçeride goto_gps_location fonksiyonunu kullanır.
        
        Parametreler:
            altitude (float): Yeni irtifa (metre, home noktasından göreceli)
        
        Döndürülenler:
            bool: True (komut gönderildi), False (hata)
        """
        # Eğer bağlantı nesnesi yoksa
        if not self.vehicle:
            # Hata mesajı yazdır
            logger.error("İrtifa değiştirmek için bağlantı yok.")
            # Başarısızlığı işaretle (False döndür)
            return False
        try:
            # Mevcut konum bilgisini al
            pos = self.get_gps()
            # Eğer konum alınamadıysa
            if not pos:
                # Hata mesajı yazdır
                logger.error("Mevcut konum alınamadı.")
                # Başarısızlığı işaretle (False döndür)
                return False
            # Aynı enlem/boylam'da, yeni irtifaya git
            self.goto_gps_location(pos["lat"], pos["lon"], altitude)
            # İrtifa değiştirme başarısını logla
            logger.info(f"İrtifa {altitude}m olarak değiştiriliyor.")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # İrtifa değiştirme sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"İrtifa değiştirme hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False

    # 26. log_mavlink_message
    def log_mavlink_message(self, msg):
        """
        Her 4 saniyede bir: tüm telemetri türlerini tek satır olarak logla.
        
        Bu fonksiyon, MAVLink mesajlarını işlediğinde tetiklenir.
        Fakat sadece 4 saniye aralıklarla log kaydı oluşturur (çok fazla 
        veri kaydını önlemek için).
        
        Parametreler:
            msg: MAVLink mesaj nesnesi (kullanılmıyor, sadece tetikleme için)
        
        Döndürülenler:
            None
        """
        try:
            # Şu anki zamanı al
            now = time.time()

            # Eğer son loglamadan itibaren log_interval kadarı geçmediyse
            if now - self.last_log_time < self.log_interval:
                # Fonksiyondan çık (log yapma)
                return

            # Log satırı: tüm telemetri türleri tek pakette
            # Bu sayede aynı anda alınan tüm veriler birlikte kaydedilir
            log_entry = {
                # Zaman: İnsan okunaklı format (YYYY-MM-DD HH:MM:SS)
                "time": datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
                # GPS ham verisi cache'i
                "gps": self._cache_gps,
                # Tutum verisi cache'i
                "attitude": self._cache_att,
                # Global konum verisi cache'i
                "global_position": self._cache_global,
                # Batarya verisi cache'i
                "battery": self._cache_battery,
                # Heartbeat verisi cache'i
                "heartbeat": self._cache_heartbeat,
            }

            # Log girdisini buffer'a ekle (JSON'a yazılmak üzere)
            self.message_buffer.append(log_entry)
            # Son log zamanını güncelle (4 saniye sonraya dek log yapılmasın)
            self.last_log_time = now

        except Exception as e:
            # Loglama sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"Log hatası: {e}")



    # Yardımcı fonksiyonlar (Helper Functions)
    
    def _to_quaternion(self, roll: float, pitch: float, yaw: float):
        """
        Roll, Pitch, Yaw açılarını quaternion formatına dönüştürür.
        
        Quaternion, 3D döndüreleri temsil etmenin matematiksel bir yoludur.
        MAVLink protokolü tutum (attitude) komutlarında quaternion formatı kullanır.
        
        Dönüştürme formülü: Euler açılarından quaternion'a
        - Roll (X ekseni): r
        - Pitch (Y ekseni): p
        - Yaw (Z ekseni): y
        
        Parametreler:
            roll (float): Roll açısı (radyan)
            pitch (float): Pitch açısı (radyan)
            yaw (float): Yaw açısı (radyan)
        
        Döndürülenler:
            list: [w, x, y, z] quaternion bileşenleri
                w: Skalir bileşen
                x, y, z: Vektör bileşenleri
        """
        # Yaw açısının yarısını trigonometrik fonksiyonlara sok
        # 0.5 ile çarpılır çünkü quaternion yarı açıları kullanır
        t0 = math.cos(yaw * 0.5)
        t1 = math.sin(yaw * 0.5)
        
        # Roll açısının yarısını trigonometrik fonksiyonlara sok
        t2 = math.cos(roll * 0.5)
        t3 = math.sin(roll * 0.5)
        
        # Pitch açısının yarısını trigonometrik fonksiyonlara sok
        t4 = math.cos(pitch * 0.5)
        t5 = math.sin(pitch * 0.5)

        # Quaternion bileşenlerini hesapla (w = skalir, x,y,z = vektör)
        # W bileşeni: Roll, Pitch, Yaw'ın kombinasyonu (skalir kısım)
        w = t0 * t2 * t4 + t1 * t3 * t5
        # X bileşeni: Roll ve Pitch bilgisi
        x = t0 * t3 * t4 - t1 * t2 * t5
        # Y bileşeni: Roll ve Pitch bilgisi
        y = t0 * t2 * t5 + t1 * t3 * t4
        # Z bileşeni: Yaw bilgisi
        z = t1 * t2 * t4 - t0 * t3 * t5
        
        # Quaternion'u liste olarak döndür
        return [w, x, y, z]

    # JSON export
    def export_message_buffer_to_json(self, filepath: str):
        """
        Loglanan tüm telemetri mesajlarını JSON dosyasına kaydeder.
        
        Bu fonksiyon, uçuş sırasında toplanan tüm telemetri verilerini
        bir JSON dosyasına yazdırır. Sonradan analiz ve debug için yararlıdır.
        
        Parametreler:
            filepath (str): Kaydedilecek dosyası yolu (örn: "logs/flight_data.json")
        
        Döndürülenler:
            bool: True (başarılı yazma), False (hata)
        """
        try:
            # Dosyayı yazma modu ("w") ve UTF-8 kodlamasında aç
            with open(filepath, "w", encoding="utf-8") as f:
                # message_buffer'daki tüm mesajları JSON formatına dönüştür ve dosyaya yaz
                # ensure_ascii=False: Türkçe karakterleri korutsun
                # indent=2: 2 boşluk girintileme (okunabilir format)
                json.dump(self.message_buffer, f, ensure_ascii=False, indent=2)
            # Başarılı yazma mesajı yazdır
            logger.info(f"Mesaj buffer JSON olarak kaydedildi: {filepath}")
            # Başarı döndür (True)
            return True
        except Exception as e:
            # JSON yazma sırasında hata oluşursa hata mesajı yazdır
            logger.error(f"JSON yazma hatası: {e}")
            # Başarısızlığı işaretle (False döndür)
            return False
