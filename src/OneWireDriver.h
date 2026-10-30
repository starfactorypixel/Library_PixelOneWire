#pragma once
#include <stdint.h>
#include <string.h>

/*
 *	Драйвер шины 1-Wire для STM32 HAL.
 *
 *	Требования:
 *	 - таймер `htim` тактируется частотой 1 МГц (1 тик = 1 мкс), запущен (HAL_TIM_Base_Start)
 *	   и больше никем не используется;
 *	 - на линии внешний подтягивающий резистор (обычно 4.7 кОм).
 *
 *	Пин постоянно работает как выход с открытым стоком: запись 0 прижимает линию к земле,
 *	запись 1 отпускает её. Читать линию можно в этом же режиме, переключать пин на вход не нужно.
 */

class OneWireDriver
{
	public:

		static constexpr uint8_t CMD_SEARCH_ROM = 0xF0;
		static constexpr uint8_t CMD_READ_ROM = 0x33;
		static constexpr uint8_t CMD_MATCH_ROM = 0x55;
		static constexpr uint8_t CMD_SKIP_ROM = 0xCC;
		static constexpr uint8_t CMD_ALARM_SEARCH = 0xEC;

		struct __attribute__((packed)) rom_t
		{
			union
			{
				uint8_t raw[8];
				struct __attribute__((packed))
				{
					uint8_t code;		// Family code
					uint8_t sn[6];
					uint8_t crc;
				};
			};
		};


		OneWireDriver(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim) :
			_port(port), _pin(pin), _htim(htim)
		{}


		// Сброс шины. true — на шине есть хотя бы одно устройство.
		bool Reset()
		{
			// Настраиваем пин при каждом сбросе: это дёшево и не зависит от того,
			// был ли включён тактовый сигнал порта на момент вызова конструктора.
			_PinInit();

			if(_ReadPin() == 0)
				return false;				// Линия прижата к земле: замыкание или нет подтяжки

			_PinLow();
			_DelayUs(480);

			uint32_t irq = _IrqOff();
			_PinHigh();
			_DelayUs(70);
			bool presence = (_ReadPin() == 0);
			_IrqOn(irq);

			_DelayUs(410);

			return presence;
		}

		// Прочитать ROM единственного устройства на шине (если устройств несколько, CRC не сойдётся)
		bool ReadRom(rom_t &rom)
		{
			if(Reset() == false)
				return false;

			WriteByte(CMD_READ_ROM);
			ReadBytes(rom.raw, sizeof(rom.raw));

			return (Crc8(rom.raw, sizeof(rom.raw)) == 0 && rom.code != 0x00);
		}

		void MatchRom(const rom_t &rom)
		{
			WriteByte(CMD_MATCH_ROM);
			WriteBytes(rom.raw, sizeof(rom.raw));

			return;
		}

		void SkipRom()
		{
			WriteByte(CMD_SKIP_ROM);

			return;
		}

		void WriteByte(uint8_t byte)
		{
			for(uint8_t i = 0; i < 8; ++i)
			{
				_WriteBit(byte & 0x01);
				byte >>= 1;
			}

			return;
		}

		uint8_t ReadByte()
		{
			uint8_t byte = 0x00;
			for(uint8_t i = 0; i < 8; ++i)
			{
				byte |= (_ReadBit() << i);
			}

			return byte;
		}

		void WriteBytes(const uint8_t *data, uint8_t length)
		{
			for(uint8_t i = 0; i < length; ++i)
			{
				WriteByte(data[i]);
			}

			return;
		}

		void ReadBytes(uint8_t *data, uint8_t length)
		{
			for(uint8_t i = 0; i < length; ++i)
			{
				data[i] = ReadByte();
			}

			return;
		}


		/*
		 *	Поиск устройств (алгоритм Maxim AN187), по одному устройству за вызов:
		 *
		 *		OneWireDriver::rom_t rom;
		 *		bus.SearchBegin();
		 *		while(bus.SearchNext(rom)) { ... }
		 *
		 *	Порядок выдачи устройств всегда одинаковый (определяется их ROM).
		 */
		void SearchBegin(bool alarm_only = false)
		{
			memset(_search_rom.raw, 0x00, sizeof(_search_rom.raw));
			_search_last = 0;
			_search_done = false;
			_search_cmd = alarm_only ? CMD_ALARM_SEARCH : CMD_SEARCH_ROM;

			return;
		}

		// Найти следующее устройство. false — устройств больше нет или ошибка на шине.
		bool SearchNext(rom_t &rom)
		{
			if(_search_done == true)
				return false;

			if(Reset() == false)
				return _SearchStop();

			WriteByte(_search_cmd);

			uint8_t last_zero = 0;		// Номер бита последнего ветвления, где пошли по нулю
			for(uint8_t bit = 1; bit <= 64; ++bit)
			{
				uint8_t &byte = _search_rom.raw[(bit - 1) / 8];
				uint8_t mask = 1 << ((bit - 1) % 8);

				uint8_t id = _ReadBit();		// Бит, который выдали устройства
				uint8_t cmp = _ReadBit();		// Его инверсия
				uint8_t dir;					// Куда идём

				if(id == 1 && cmp == 1)
					return _SearchStop();		// Никто не ответил

				if(id != cmp)
				{
					dir = id;					// У всех оставшихся устройств этот бит одинаковый
				}
				else
				{
					// Ветвление: есть устройства и с 0, и с 1
					if(bit < _search_last)
						dir = (byte & mask) ? 1 : 0;		// До прошлой точки ветвления повторяем прежний путь
					else
						dir = (bit == _search_last) ? 1 : 0;	// В ней поворачиваем на 1, дальше всегда на 0

					if(dir == 0)
						last_zero = bit;				// Сюда вернёмся на следующем проходе
				}

				if(dir) byte |= mask;
				else byte &= ~mask;

				_WriteBit(dir);
			}

			// CRC от восьми нулей тоже равна 0, поэтому отдельно отсекаем пустой ROM
			if(Crc8(_search_rom.raw, sizeof(_search_rom.raw)) != 0 || _search_rom.code == 0x00)
				return _SearchStop();

			_search_last = last_zero;
			_search_done = (last_zero == 0);
			rom = _search_rom;

			return true;
		}

		// Найти все устройства сразу (блокирующе). Возвращает количество найденных.
		template <uint8_t N>
		uint8_t SearchAll(rom_t (&roms)[N], bool alarm_only = false)
		{
			uint8_t count = 0;

			SearchBegin(alarm_only);
			while(count < N && SearchNext(roms[count]))
			{
				count++;
			}

			return count;
		}


		static uint8_t Crc8(const uint8_t *data, uint8_t length)
		{
			uint8_t crc = 0x00;

			while(length--)
			{
				uint8_t inbyte = *data++;
				for(uint8_t i = 8; i; i--)
				{
					uint8_t mix = (crc ^ inbyte) & 0x01;
					crc >>= 1;
					if(mix) crc ^= 0x8C;
					inbyte >>= 1;
				}
			}

			return crc;
		}

	private:

		void _WriteBit(uint8_t bit)
		{
			// Длительность низкого уровня критична (особенно 6 мкс для «1»), поэтому без прерываний
			uint32_t irq = _IrqOff();
			_PinLow();
			if(bit)
			{
				_DelayUs(6);
				_PinHigh();
				_IrqOn(irq);
				_DelayUs(64);
			}
			else
			{
				_DelayUs(60);
				_PinHigh();
				_IrqOn(irq);
				_DelayUs(10);
			}

			return;
		}

		uint8_t _ReadBit()
		{
			// Линию нужно прочитать не позже 15 мкс от начала слота, поэтому без прерываний
			uint32_t irq = _IrqOff();
			_PinLow();
			_DelayUs(6);
			_PinHigh();
			_DelayUs(9);
			uint8_t bit = _ReadPin();
			_IrqOn(irq);

			_DelayUs(55);

			return bit;
		}

		void _DelayUs(uint16_t us)
		{
			__HAL_TIM_SET_COUNTER(_htim, 0);
			while(__HAL_TIM_GET_COUNTER(_htim) < us)
			{
				__NOP();
			}

			return;
		}

		void _PinInit()
		{
			// Сначала 1 в выходной регистр, чтобы при переключении режима не было импульса на линии
			_PinHigh();

			GPIO_InitTypeDef cfg = {};
			cfg.Pin = _pin;
			cfg.Mode = GPIO_MODE_OUTPUT_OD;
			cfg.Pull = GPIO_NOPULL;
			cfg.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(_port, &cfg);

			return;
		}

		void _PinLow()
		{
			HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
		}

		// Открытый сток: 1 в выходной регистр отпускает линию, высокий уровень даёт внешняя подтяжка
		void _PinHigh()
		{
			HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
		}

		uint8_t _ReadPin()
		{
			return (HAL_GPIO_ReadPin(_port, _pin) == GPIO_PIN_SET) ? 1 : 0;
		}

		// Запрет прерываний с сохранением прежнего состояния: безопасно, даже если они уже были запрещены.
		// Сохранённое значение компилятор держит в регистре, на стек оно не попадает. Хранить его в поле
		// класса невыгодно: это лишние str/ldr в память на каждом бите (проверено на Cortex-M3, -Os: +20 байт кода).
		static uint32_t _IrqOff()
		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();

			return primask;
		}

		static void _IrqOn(uint32_t primask)
		{
			__set_PRIMASK(primask);
		}

		bool _SearchStop()
		{
			_search_done = true;

			return false;
		}


		GPIO_TypeDef *_port;
		uint16_t _pin;
		TIM_HandleTypeDef *_htim;

		rom_t _search_rom;
		uint8_t _search_last = 0;
		uint8_t _search_cmd = CMD_SEARCH_ROM;
		bool _search_done = true;
};
