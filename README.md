# MT6701-Arduino-Library
<p align="center"><img src="/images/mt6701_module.jpg"></p>

## Еще магнытные датчики
* [AS5600](https://github.com/S-LABc/AMS-AS5600-Arduino-Library)
* [AS5601](https://github.com/S-LABc/AMS-AS5601-Arduino-Library)

## Предупреждение
* Библиотека не проверялась полностью. Возможны ошибки. Используя бибилиотеку вы берете все риски на себя
* Реализованы не все возможности датчика. Для взаимодействия с регистрами датчика можно использовать эти методы:
```C++
// Чтение содержимого регистра
uint8_t MT_RequestSingleRegister(uint8_t _reg_addr);
// Запись нового содержимого в регистр
void MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload);
```
* Все доступные методы, константы, типы данных можно посмотреть [тут](https://github.com/S-LABc/MT6701-Arduino-Library/blob/main/src/MT6701_I2C.h)

## Ссылки
* [Даташит MT6701](http://www.magntek.com.cn/upload/MT6701_Rev.1.5.pdf)
* [Страница MT6701](http://www.magntek.com.cn/en/list/177/559.htm)
* [Корпус для тестов](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/tree/main/addons/AS5600-Case-STL)
* [Видео с моим мнением](https://youtu.be/GnFONlue2t0)