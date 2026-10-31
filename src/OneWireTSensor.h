#pragma once
#include <stdint.h>
#include "OneWireDriver.h"

/*
 *	Один датчик температуры DS18x20 с известным ROM: конвертация, чтение, фильтр сбоев.
 *	Работает самостоятельно или в составе OneWireTSens.
 *
 *		static const OneWireDriver::rom_t ROM_OUTSIDE = {{0x28, 0xFF, 0x64, 0x1E, 0x15, 0x21, 0x03, 0x8C}};
 *		OneWireTSensor outside(bus, ROM_OUTSIDE);
 *
 *		outside.Convert();
 *		HAL_Delay(OneWireTSensor::CONVERT_TIME);
 *		if(outside.Read()) temp = outside.GetTemp();
 *
 *	Если датчик на шине один, ROM можно узнать через bus.ReadRom(rom).
 *	Температура в сотых долях градуса: 2537 = 25.37 °C.
 */

class OneWireTSensor
{
	public:

		static constexpr uint8_t CMD_CONVERT_T = 0x44;
		static constexpr uint8_t CMD_WRITE_SCRATCHPAD = 0x4E;
		static constexpr uint8_t CMD_READ_SCRATCHPAD = 0xBE;
		static constexpr uint8_t CMD_COPY_SCRATCHPAD = 0x48;
		static constexpr uint8_t CMD_RECALL_E2 = 0xB8;
		static constexpr uint8_t CMD_READ_POWER_SUPPLY = 0xB4;

		// Содержимое памяти датчика (9 байт), порядок байт little-endian, как и на STM32
		struct __attribute__((packed)) scratchpad_t
		{
			union
			{
				uint8_t raw[9];
				struct __attribute__((packed))
				{
					int16_t temp;			// Температура (DS18B20: 1/16 °C, DS18S20: 1/2 °C)
					uint8_t th;				// Порог тревоги, верхний
					uint8_t tl;				// Порог тревоги, нижний
					uint8_t cfg;			// DS18B20: разрешение; DS18S20: зарезервирован (0xFF)
					uint8_t reserved[3];	// DS18S20: [1] = COUNT_REMAIN, [2] = COUNT_PER_C
					uint8_t crc;			// CRC8 первых 8 байт
				};
			};
		};
		static_assert(sizeof(scratchpad_t) == 9, "scratchpad_t must be 9 bytes");

		static constexpr int16_t NO_VALID_TEMP = INT16_MAX;
		static constexpr int8_t NO_VALID_TEMP8 = (int8_t)(NO_VALID_TEMP >> 8);

		// Метка «датчик не перезагружался» в TH/TL. +127 / -128 °C — пороги, при которых тревога
		// никогда не срабатывает, и маловероятно, что такие же значения записаны в EEPROM.
		static constexpr uint8_t MARK_TH = 0x7F;
		static constexpr uint8_t MARK_TL = 0x80;

		// Конфигурация DS18B20 / DS1822: 0 R1 R0 1 1 1 1 1. Пишется вместе с меткой, только в память датчика,
		// поэтому настройка в EEPROM не меняется. 0x1F = 9 бит, 0x3F = 10, 0x5F = 11, 0x7F = 12 бит.
		static constexpr uint8_t CFG_12BIT = 0x7F;
		static constexpr uint16_t CONVERT_TIME = 800;		// мс, 12 бит = 750 мс + запас

		// Фильтр сбоев: столько неудачных чтений подряд прощается (отдаётся последнее верное
		// значение), после этого датчик считается неисправным до первого удачного чтения.
		static constexpr uint8_t ERROR_LIMIT = 3;


		// Пустой датчик (нужен для массивов): не привязан к шине, всегда невалиден
		constexpr OneWireTSensor() :
			_driver(nullptr), _rom(), _temp(NO_VALID_TEMP), _errors(ERROR_LIMIT)
		{}

		// Если ROM не от датчика температуры, датчик не привязывается к шине и ведёт себя как пустой.
		// Так семейство проверяется один раз здесь, а не при каждом чтении.
		OneWireTSensor(OneWireDriver &driver, const OneWireDriver::rom_t &rom) :
			_driver(IsSupported(rom) ? &driver : nullptr), _rom(rom), _temp(NO_VALID_TEMP), _errors(ERROR_LIMIT)
		{}


		// Запустить конвертацию на этом датчике. Результат готов через CONVERT_TIME.
		bool Convert()
		{
			if(_driver == nullptr || _driver->Reset() == false)
				return false;

			_driver->MatchRom(_rom);
			_driver->WriteByte(CMD_CONVERT_T);

			return true;
		}

		// Прочитать датчик и обновить показания. true — чтение удачное.
		bool Read()
		{
			int16_t temp;
			if(_ReadTemp(temp) == true)
			{
				_temp = temp;
				_errors = 0;

				return true;
			}

			if(_errors < ERROR_LIMIT)
				_errors++;

			return false;
		}


		// Общий пустой датчик (лежит во flash, один на всю прошивку). Его возвращает
		// OneWireTSens::operator[] при неверном индексе вместо чтения за пределами массива.
		static const OneWireTSensor &Empty()
		{
			static constexpr OneWireTSensor empty;
			return empty;
		}

		// true — датчик привязан к шине (ROM от поддерживаемого датчика)
		bool IsBound() const
		{
			return (_driver != nullptr);
		}

		bool IsValid() const
		{
			return (_errors < ERROR_LIMIT);
		}

		// Последняя верная температура или NO_VALID_TEMP
		int16_t GetTemp() const
		{
			return IsValid() ? _temp : NO_VALID_TEMP;
		}

		// Последняя верная температура в целых градусах (с округлением) или NO_VALID_TEMP8
		int8_t GetTemp8() const
		{
			if(!IsValid())
				return NO_VALID_TEMP8;

			return (int8_t)((_temp >= 0) ? (_temp + 50) / 100 : (_temp - 50) / 100);
		}

		const OneWireDriver::rom_t &GetRom() const
		{
			return _rom;
		}

		static bool IsSupported(const OneWireDriver::rom_t &rom)
		{
			switch(rom.code)
			{
				case 0x10:		// DS1820, DS18S20
				case 0x22:		// DS1822
				case 0x28:		// DS18B20
					return true;
				default:
					return false;
			}
		}

	private:

		bool _ReadTemp(int16_t &temp)
		{
			if(_driver == nullptr || _driver->Reset() == false)
				return false;

			scratchpad_t sp;
			_driver->MatchRom(_rom);
			_driver->WriteByte(CMD_READ_SCRATCHPAD);
			_driver->ReadBytes(sp.raw, sizeof(sp.raw));

			if(OneWireDriver::Crc8(sp.raw, sizeof(sp.raw) - 1) != sp.crc)
				return false;

			// Девять нулей проходят проверку CRC: это не датчик, а прижатая к земле линия.
			// У исправного датчика старшие биты cfg (DS18B20) или зарезервированный байт (DS18S20) равны 1.
			if(sp.cfg == 0x00 && sp.reserved[0] == 0x00)
				return false;

			int16_t raw = sp.temp;

			const bool is_s20 = (_rom.code == 0x10);

			// Метка — TH, TL и (у DS18B20 / DS1822) разрешение 12 бит. Её нет, если датчик перезагружался
			// по питанию, опрашивается впервые или его кто-то перенастроил. Тогда ставим метку заново
			// и проверяем, не пришло ли значение 85.00 °C, которое датчик выдаёт после включения без
			// конвертации. Если метка на месте, 85.00 °C — настоящая температура.
			if(sp.th != MARK_TH || sp.tl != MARK_TL || (is_s20 == false && sp.cfg != CFG_12BIT))
			{
				_WriteMark();

				if(raw == (is_s20 ? 0x00AA : 0x0550))
					return false;
			}

			if(is_s20)
			{
				temp = (int32_t)raw * 100 / 2;		// DS18S20: шаг 0.5 °C
			}
			else
			{
				// При разрешении ниже 12 бит младшие биты не определены (бывает в одном чтении,
				// если датчик перезагрузился перед конвертацией). При 12 битах маска ничего не меняет.
				uint8_t undefined_bits = 3 - ((sp.cfg >> 5) & 0x03);
				raw &= ~((1 << undefined_bits) - 1);

				temp = (int32_t)raw * 100 / 16;		// DS18B20, DS1822: шаг 0.0625 °C
			}

			return true;
		}


		// Записать метку в TH/TL и разрешение 12 бит командой Write Scratchpad. Без Copy Scratchpad
		// это попадает только в память датчика, не в EEPROM, поэтому после сброса по питанию датчик
		// загрузит TH/TL/cfg из EEPROM и метка пропадёт. Все байты передаются подряд до следующего сброса.
		void _WriteMark()
		{
			if(_driver->Reset() == false)
				return;

			_driver->MatchRom(_rom);
			_driver->WriteByte(CMD_WRITE_SCRATCHPAD);
			_driver->WriteByte(MARK_TH);
			_driver->WriteByte(MARK_TL);
			if(_rom.code != 0x10)
				_driver->WriteByte(CFG_12BIT);		// DS18B20, DS1822: третий байт — конфигурация

			return;
		}


		OneWireDriver *_driver;
		OneWireDriver::rom_t _rom;
		int16_t _temp;			// Последняя верная температура, сотые градуса
		uint8_t _errors;		// Неудачных чтений подряд (не больше ERROR_LIMIT)
};
