# TRIAD Algoritması Örnekleri (Türkçe Açıklamalı)

Bu depo, uydu/CubeSat **yönelim belirleme (attitude determination)**
problemlerinde klasik olarak kullanılan **TRIAD algoritmasını** sıfırdan
anlatan, çalışan kod örnekleri ve matematiksel türetim içeren bir
kaynaktır. Temel algoritmadan gerçekçi bir Güneş sensörü + magnetometre
uygulamasına, kuaterniyon dönüşümünden hata/gürültü analizine kadar
uzanan bir seri içerir.

Bu depo, [`pwm-ornekleri`](../pwm-ornekleri) ve B-dot detumbling örneğiyle
aynı ADCS (Attitude Determination and Control System) ailesindendir:
B-dot uydunun **dönüşünü söndürürken**, TRIAD uydunun **o anki tam
yönelimini** hesaplar.

Daha formal/matematiksel anlatım için: [`docs/triad_anlatim.tex`](docs/triad_anlatim.tex)
(LaTeX -> PDF, depoda derlenmiş hali de mevcut: [`docs/triad_anlatim.pdf`](docs/triad_anlatim.pdf)).

---

## İçindekiler

1. [TRIAD Nedir?](#triad-nedir)
2. [Neden İki Vektör Gerekir?](#neden-iki-vektor-gerekir)
3. [Algoritmanın Özeti](#algoritmanin-ozeti)
4. [Depo Yapısı](#depo-yapisi)
5. [Örnekler](#ornekler)
   - [01 - Temel TRIAD (C)](#01---temel-triad-c)
   - [02 - Python/NumPy Doğrulama](#02---pythonnumpy-dogrulama)
   - [03 - DCM'den Kuaterniyona Dönüşüm](#03---dcmden-kuaterniyona-donusum)
   - [04 - Güneş Sensörü + Magnetometre (Gerçekçi Örnek)](#04---gunes-sensoru--magnetometre-gercekci-ornek)
   - [05 - Hata ve Gürültü Analizi](#05---hata-ve-gurultu-analizi)
6. [Sık Yapılan Hatalar](#sik-yapilan-hatalar)
7. [Lisans](#lisans)

---

## TRIAD Nedir?

TRIAD, iki farklı vektör ölçümünden bir cismin (örneğin bir uydunun)
yönelimini (attitude) **kapalı formda, iterasyonsuz** olarak hesaplayan
klasik bir algoritmadır. Fikir şudur:

Aynı iki fiziksel yönü (örneğin Güneş yönü ve Dünya'nın manyetik alan
yönü) **iki farklı çerçevede** biliyoruz:

- **Referans çerçevesi**: Bu vektörlerin bilinen/hesaplanan değerleri (Güneş efemerisi, IGRF manyetik alan modeli).
- **Gövde (body) çerçevesi**: Bu vektörlerin uydu üzerindeki sensörlerle **ölçülen** değerleri.

Gövde çerçevesindeki bu vektörleri referans çerçevesindeki karşılıklarına
"döndürecek" $A$ rotasyon matrisini (DCM) bulursak, bu matris uydunun o
anki **yönelimini** verir.

## Neden İki Vektör Gerekir?

Tek bir vektör eşleşmesi, o vektör ekseni etrafındaki dönmeyi (roll)
belirsiz bırakır — sonsuz sayıda geçerli çözüm vardır. 3 serbestlik
derecesine sahip bir yönelimi tam belirlemek için **birbirinden bağımsız
(paralel olmayan) en az iki vektör** ölçümü gerekir.

## Algoritmanın Özeti

```
r1, r2   : Referans cercevesindeki iki bilinen birim vektor
b1, b2   : Govde cercevesinde OLCULEN karsiliklari

t1r = r1 / |r1|              t1b = b1 / |b1|
t2r = (r1 x r2) / |r1 x r2|  t2b = (b1 x b2) / |b1 x b2|
t3r = t1r x t2r              t3b = t1b x t2b

Mr = [t1r t2r t3r]           Mb = [t1b t2b t3b]

A = Mb * Mr^T   <-- Yonelim DCM'i (referans -> govde)
```

**Önemli:** TRIAD asimetriktir. $(\vec{r}_1, \vec{b}_1)$ çifti tam
korunur, $(\vec{r}_2, \vec{b}_2)$ yalnızca "yardımcı" olarak kullanılır
ve bilgisinin bir kısmı kaybolur. Bu yüzden **en güvenilir/en az
gürültülü sensör her zaman birincil vektör** ($\vec{r}_1,\vec{b}_1$)
olarak seçilmelidir (örneğin Güneş sensörü genelde magnetometreden daha
doğrudur).

## Depo Yapısı

```
triad-ornekleri/
├── README.md
├── LICENSE
├── docs/
│   ├── triad_anlatim.tex      <- LaTeX aciklama dokumani
│   └── triad_anlatim.pdf      <- Derlenmis PDF
└── examples/
    ├── 01_triad_temel/
    ├── 02_triad_python_dogrulama/
    ├── 03_dcm_to_quaternion/
    ├── 04_gunes_magnetometre_ornegi/
    └── 05_hata_gurultu_analizi/
```

## Örnekler

### 01 - Temel TRIAD (C)
`examples/01_triad_temel/triad_basic.c`

Algoritmanın çekirdek matematiğini, harici kütüphane kullanmadan (yalnızca
`math.h`), adım adım yorumlarla anlatan referans C implementasyonu.
Kendi vektör/matris yardımcı fonksiyonlarını içerir (`vec_cross`,
`vec_normalize`, `mat_mult_transpose` vb.) ve sonunda $A \cdot r_1 = b_1$
eşitliğiyle kendi kendini doğrular.

```bash
cd examples/01_triad_temel
gcc triad_basic.c -o triad_test -lm
./triad_test
```

### 02 - Python/NumPy Doğrulama
`examples/02_triad_python_dogrulama/triad_verify.py`

C implementasyonunu bağımsız bir dille (NumPy) doğrulamak ve farklı
senaryoları hızlıca denemek için yazılmıştır. Üç senaryo içerir:

1. Gürültüsüz, bilinen bir rotasyonla tam doğrulama.
2. Rastgele 3 eksenli rotasyon + %1 sensör gürültüsü ile gerçekçi hata ölçümü.
3. **Birincil vektör seçiminin sonucu nasıl etkilediğinin gösterimi** (r1 ↔ r2 rolleri değiştirildiğinde çıkan farkı ölçer).

```bash
pip install numpy
python3 examples/02_triad_python_dogrulama/triad_verify.py
```

### 03 - DCM'den Kuaterniyona Dönüşüm
`examples/03_dcm_to_quaternion/dcm_to_quaternion.c`

TRIAD çıktısı olan DCM'i, gerçek ADCS yazılımlarının kullandığı
kuaterniyon formuna çeviren, sayısal olarak kararlı (en büyük diyagonal
elemana göre dallanan, Shepperd yöntemine benzer) bir dönüşüm. Ayrıca
kuaterniyondan geri DCM'e ve Euler açılarına dönüşüm de gösterilmiştir.

```bash
cd examples/03_dcm_to_quaternion
gcc dcm_to_quaternion.c -o q_test -lm
./q_test
```

### 04 - Güneş Sensörü + Magnetometre (Gerçekçi Örnek)
`examples/04_gunes_magnetometre_ornegi/gunes_magnetometre.c`

**Depodaki en kapsamlı örnek.** Gerçek bir CubeSat ADCS pipeline'ını
uçtan uca simüle eder:

- Güneş sensörü ve magnetometre okumalarını (simüle edilmiş) alır,
- Güneş vektörünü **birincil** (daha güvenilir), manyetik alan vektörünü
  **ikincil** olarak TRIAD'a verir,
- **Eclipse (Dünya gölgesi) kontrolü** içerir: Güneş sensörü geçersiz veri
  üretiyorsa TRIAD yerine son bilinen yönelim + jiroskop propagasyonuna
  geçilmesi gerektiğini not eder,
- Hesaplanan DCM'i, referans Güneş vektörünü geri döndürerek doğrular.

```bash
cd examples/04_gunes_magnetometre_ornegi
gcc gunes_magnetometre.c -o gercekci_ornek -lm
./gercekci_ornek
```

Gerçek donanıma taşırken sensör okuma fonksiyonlarını (I2C/SPI) ve
efemeris/IGRF hesaplarını doldurmanız gerektiği dosya içinde belirtilmiştir.

### 05 - Hata ve Gürültü Analizi
`examples/05_hata_gurultu_analizi/noise_analysis.py`

Monte Carlo simülasyonlarıyla TRIAD doğruluğunu etkileyen iki temel
faktörü sayısal olarak inceler:

- **Sensör gürültüsü** arttıkça yönelim hatasının yaklaşık doğrusal arttığını,
- **Referans vektörler arası açı** 90°'den uzaklaştıkça (paralele
  yaklaştıkça) hatanın hızla büyüdüğünü ("U şeklinde" davranış)

500 denemelik istatistiklerle gösterir.

```bash
pip install numpy
python3 examples/05_hata_gurultu_analizi/noise_analysis.py
```

**Özet sonuçlar (depodaki gerçek çalıştırmadan):**

| Gürültü σ | Ort. hata (derece) |
|-----------|---------------------|
| 0.000     | 0.000               |
| 0.010     | 0.903               |
| 0.050     | 4.529               |

| r1-r2 açısı | Ort. hata (derece) |
|-------------|---------------------|
| 10°         | 3.97                |
| 90°         | 0.92 (en iyi)       |
| 170°        | 3.77                |

## Sık Yapılan Hatalar

- **En güvenilmez sensörü birincil vektör seçmek**: TRIAD asimetriktir;
  daima en düşük gürültülü ölçümü $(\vec{r}_1,\vec{b}_1)$ yapın.
- **Vektörleri normalize etmeyi unutmak**: Tüm girdi vektörleri birim
  vektör olmalıdır, aksi halde çıkan matris geçerli bir rotasyon
  matrisi (ortonormal, $\det=1$) olmaz.
- **Neredeyse paralel referans vektörleri kullanmak**: Çapraz çarpımın
  büyüklüğü küçüldükçe gürültü aşırı büyütülür; mümkün olduğunca dike
  yakın (90°) vektör çiftleri tercih edin.
- **Eclipse durumunu göz ardı etmek**: Güneş sensörü, uydu Dünya
  gölgesindeyken geçersiz/gürültülü veri üretir; bu durumda TRIAD yerine
  son bilinen yönelim + jiroskop entegrasyonuna geçilmelidir.
- **DCM'yi doğrudan kontrolcüye vermek**: Çoğu kontrol algoritması
  kuaterniyon veya açısal hata metriği bekler; önce dönüştürmeyi
  unutmayın (bkz. Örnek 03).

## Lisans

MIT Lisansı altında paylaşılmıştır - bkz. [`LICENSE`](LICENSE).
