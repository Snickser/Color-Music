# Color-Music
Arduino Color-Music is an audio reactive VU Meter for WS181x LED Strip.

Now 11 (6 centered, 3 linear, 2 EQ) animated effects.

It uses:
- NeoPixelBus library https://github.com/Makuna/NeoPixelBus
- FHT library http://wiki.openmusiclabs.com/wiki/ArduinoFHT
- <i>For old release ArduinoFFT</i> https://github.com/kosme/arduinoFFT
- EncButton library https://github.com/GyverLibs/EncButton
- Mic on MAX9814 amplifier (or any similar), or any line-input audio signal.
- EC11 encoder for switch modes, brightness and sensitivity.
- Arduino Nano(328p)/ProMicro(32U4) (or any other where the specified libraries work).

Список эффектов:
1. Центральный, фиксированные цвета.
2. Линейный, фиксированные цвета.
3. Центральный, фиксированные цвета, три сегмента.
4. Линейный, цвет меняется последовательно со случайным направлением палитры.
5. Эквалайзер, попиксельный из центра, 62 полосы.
6. Центральный, бегущая радуга со случайным направлением палитры.
7. Эквалайзер, 5 полос.
8. Поток радуги из центра.
9. Центральный, случайный цвет.
10. Линейный, бегущая радуга со случайным направлением палитры.
11. Центральный, смешение двух случайных цветов.

Кнопка переключает последовательно четыре режима настроек, а крутилка меняет значение -/+
1. Ручное переключение эффекта (по умолчанию)
2. Изменение яркости (1/Br)
3. Изменение времени обработки звука (в миллисекундах, с отображением линейки)
4. Изменяет коэффициент увеличения верхнего предела уровня звука (чтоб не зашкаливало, обычно не требуется, на всякий случай)
5. Изменяет скорости падения пик-маркера.

<b>! Количество пикселей в ленте должно быть кратно 2, 5, 9, или 10. В остальных случаях возможны глюки.</b>
