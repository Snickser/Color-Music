[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://paypal.me/snickser) support me with a donation.

# Welcome to my project - Color-Music
Arduino Color-Music is an audio reactive VU Meter for WS181x LED strip.

Now 15 (9 centered, 3 linear, 2 EQ, and flash) animated effects.

<b>Software and hardware:</b>
- NeoPixelBus library https://github.com/Makuna/NeoPixelBus
- FHT library http://wiki.openmusiclabs.com/wiki/ArduinoFHT <i>(not compatible for ESPxx)</i>
- ArduinoFFT https://github.com/kosme/arduinoFFT <i>(in old release for ESP8266/ESP32)</i>
- EncButton library https://github.com/GyverLibs/EncButton
- Mic on MAX9814 amplifier (or any similar), or any line-input audio signal.
- EC11 encoder for switch modes, brightness and sensitivity (and etc.).
- Arduino Nano (or any other where the specified libraries work).

<b>Список эффектов:</b>
1. Центральные (1-5), фиксированные цвета.
2. Линейный, фиксированные цвета.
3. Центральный, фиксированные цвета, три сегмента.
4. Линейный, цвет меняется последовательно со случайным направлением палитры.
5. Эквалайзер, попиксельный из центра, 62 полосы.
6. Центральный (1 или 2), бегущая радуга со случайным направлением палитры.
7. Эквалайзер, 5 полос.
8. Поток радуги из центра.
9. Центральный, случайный цвет.
10. Линейный, бегущая радуга со случайным направлением палитры.
11. Центральный, смешение двух случайных цветов.
12. Случайные вспышки по всей ленте.
13. Центральный, уровень с цветом + бегунок при нуле.
14. Центральный (2-5) как 9 или 11, случайно выпадает.
15. Центральный, цвет меняется последовательно со случайным направлением палитры.
16. Случайный цвет каждого пикселя.

<b>Кнопка переключает последовательно пять режимов настроек, а крутилка меняет значение -/+</b><br>
1. Ручное переключение эффекта (по умолчанию).
2. Изменение яркости.
3. Изменение времени обработки звука (в миллисекундах, с отображением линейки).
4. Изменяет коэффициент увеличения верхнего предела уровня звука (чтоб не зашкаливало, обычно не требуется, на всякий случай).
5. Изменяет скорость падения пик-маркера.
6. Переключение режима аппаратного усиления микрофона (40/50 dВ)

Настройки (кроме третьей) сохраняются в EEPROM и восстанавливаются после подачи питания.

Пример работы:
1. https://youtu.be/ZGHia4QcqCc
2. https://youtu.be/5XOJjzPNlhk

Картинки:

![img](20210723_191054p.jpg)
![img](20210811_212356.jpg)
![img](photo_2021-12-29_19-46-13.jpg)
