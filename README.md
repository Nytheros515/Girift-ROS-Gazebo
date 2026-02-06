Girift-ROS-Gazebo: İHA Kamera Entegrasyonu ve Otonom Görev Simülasyonu
Bu proje, Gazebo simülasyon ortamında ArduPilot ve ROS 2 Humble kullanarak bir İHA'ya (Iris Copter) kamera entegrasyonu yapılması, canlı görüntü işlenmesi ve otonom görev icrasını içerir. Adana Alparslan Türkeş Bilim ve Teknoloji Üniversitesi'ndeki çalışmalarım kapsamında geliştirilmiştir.

🚀 Yapılan Çalışmalar
Donanım Modifikasyonu: Gazebo üzerindeki iris_with_ardupilot modeline ROS 2 uyumlu bir kamera sensörü eklendi ve kameranın konumu dronun altındaki orijinal gimbal yuvasına sabitlendi.

Canlı Görüntü Aktarımı: /webcam/camera/image_raw topic'i üzerinden OpenCV kütüphanesi kullanılarak canlı görüntü akışı sağlandı.

Otonom Kontrol: ArduPilot üzerinden kare_yaw görevi ile İHA'nın kare çizerek kendi ekseni etrafında tarama yapması sağlandı.

Simülasyon Ortamı: iris_arducopter_runway.world dosyası modifiye edilerek hayalet modellerden temizlendi ve stabil bir uçuş pisti oluşturuldu.

🛠️ Kurulum ve Çalıştırma
Projeyi çalıştırmak için sırasıyla aşağıdaki terminalleri açın:

1. Terminal: Gazebo Simülasyonu
Bash
source /opt/ros/humble/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib
# Modifiye edilmiş pisti başlatır
gazebo --verbose -s libgazebo_ros_factory.so /usr/share/gazebo-11/worlds/iris_arducopter_runway.world

2. Terminal: Dronun Sahneye Çağrılması (Spawn)
Bash
source /opt/ros/humble/setup.bash
# Kamerası modifiye edilmiş modeli pistin ortasına ışınlar
ros2 run gazebo_ros spawn_entity.py -file models/iris_with_ardupilot/model.sdf -entity benim_dronum -x 0 -y 0 -z 0.1

3. Terminal: ArduPilot SITL Kontrolü
Bash
cd ~/ardupilot/ArduCopter
# Dronun beynini ve bağlantı arayüzünü açar
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

4. Terminal: Kamera Görüntüleme ve Görev
Bash
# Canlı görüntü penceresini ve OpenCV akışını açmak için:
python3 scripts/kamera_oku.py

# Otonom kare çizme ve yaw görevini başlatmak için (yeni terminalde):
python3 scripts/kare_yaw.py
📁 Proje Yapısı
models/: Kamera entegreli SDF model ve config dosyaları.

scripts/: Kamera verisi okuma ve otonom uçuş algoritmalarını içeren Python kodları.

worlds/: Temizlenmiş pist, ışıklandırma ve gölge ayarlarını içeren dünya dosyası.
