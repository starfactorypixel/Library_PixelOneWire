#pragma once
#include <stdint.h>
#include "OneWireDriver.h"
#include "OneWireTSensor.h"

/*
 *	Датчики температуры DS18x20 на одной шине 1-Wire: таблица датчиков, поиск, опрос.
 *
 *	Неблокирующий режим (вызывать в главном цикле):
 *
 *		OneWireDriver bus(GPIOB, GPIO_PIN_9, &htim1);
 *		OneWireTSens<16> sensors(bus);
 *
 *		if(sensors.Processing(HAL_GetTick()))
 *		{
 *			for(uint8_t i = 0; i < sensors.Count(); ++i)
 *			{
 *				if(sensors[i].IsValid() == false) continue;
 *				temp[i] = sensors[i].GetTemp() / 100;
 *			}
 *		}
 *
 *	Блокирующий режим:
 *
 *		sensors.Search();
 *		sensors.Convert();
 *		HAL_Delay(800);
 *		sensors.ReadAll();
 *
 *	Необязательный колбэк вызывается каждый раз, когда прочитаны все датчики
 *	(в Processing() и в ReadAll()). Лямбда без захвата подходит:
 *
 *		sensors.RegReadyCallback([](const OneWireTSensor *sensor, uint8_t count)
 *		{
 *			for(uint8_t i = 0; i < count; ++i)
 *			{
 *				if(sensor[i].IsValid() == false) continue;
 *				temp[i] = sensor[i].GetTemp() / 100;
 *			}
 *		});
 *
 *	Неиспользуемый режим в прошивку не попадает (методы шаблона компилируются только при вызове).
 */

template <uint8_t _max_sensor>
class OneWireTSens
{
	public:

		static constexpr int16_t NO_VALID_TEMP = OneWireTSensor::NO_VALID_TEMP;
		static constexpr uint16_t RETRY_TIME = 5000;		// мс, повтор поиска, если датчики не найдены


		using callback_ready_t = void (*)(const OneWireTSensor *sensor, uint8_t count);


		// period — период опроса в неблокирующем режиме, мс
		OneWireTSens(OneWireDriver &driver, uint16_t period = 2000) :
			_driver(driver), _count(0), _callback_ready(nullptr),
			_period(period), _state(STATE_SEARCH_BEGIN), _idx(0), _timer(0)
		{}

		// Необязательный колбэк «прочитаны все датчики». nullptr — отключить.
		void RegReadyCallback(callback_ready_t callback)
		{
			_callback_ready = callback;

			return;
		}


		uint8_t Count() const
		{
			return _count;
		}

		// Датчик по индексу, только чтение. При неверном индексе — пустой (всегда невалидный) датчик.
		const OneWireTSensor &operator[](uint8_t idx) const
		{
			return (idx < _count) ? _sensors[idx] : OneWireTSensor::Empty();
		}


		// ---- Неблокирующий режим ----

		// true — опрошены все датчики, свежие данные в таблице.
		// За один вызов не больше одной операции на шине: поиск одного устройства (~15 мс),
		// чтение одного датчика (~12 мс) или запуск конвертации (~2 мс).
		bool Processing(uint32_t time)
		{
			switch(_state)
			{
				case STATE_SEARCH_BEGIN:
				{
					_SearchBegin();
					_state = STATE_SEARCH;

					break;
				}

				case STATE_SEARCH:
				{
					if(_SearchStep() == true)
						break;		// Ищем по одному устройству за вызов

					_timer = time;
					_state = (_count > 0) ? STATE_CONVERT : STATE_NO_SENSOR;

					break;
				}

				case STATE_NO_SENSOR:
				{
					if(time - _timer >= RETRY_TIME)
						_state = STATE_SEARCH_BEGIN;

					break;
				}

				case STATE_CONVERT:
				{
					// Если шина не ответила — не страшно: чтения не пройдут и сработает фильтр сбоев
					Convert();
					_state = STATE_CONVERT_WAIT;

					break;
				}

				case STATE_CONVERT_WAIT:
				{
					if(time - _timer < OneWireTSensor::CONVERT_TIME)
						break;

					_idx = 0;
					_state = STATE_READ;

					break;
				}

				case STATE_READ:
				{
					_sensors[_idx].Read();
					if(++_idx < _count)
						break;		// Читаем по одному датчику за вызов

					_ReadyNotify();
					_state = STATE_PAUSE;

					return true;
				}

				case STATE_PAUSE:
				{
					if(time - _timer < _period)
						break;		// Период отсчитывается от прошлого запуска конвертации

					_timer = time;
					_state = STATE_CONVERT;

					break;
				}
			}

			return false;
		}

		// Перезапустить поиск датчиков (например, после замены датчика)
		void Rescan()
		{
			_state = STATE_SEARCH_BEGIN;

			return;
		}


		// ---- Блокирующий режим ----

		// Найти все датчики. Остальные устройства на шине пропускаются.
		uint8_t Search()
		{
			_SearchBegin();
			while(_SearchStep() == true) {}

			return _count;
		}

		// Запустить конвертацию на всех датчиках сразу
		bool Convert()
		{
			if(_driver.Reset() == false)
				return false;

			_driver.SkipRom();
			_driver.WriteByte(OneWireTSensor::CMD_CONVERT_T);

			return true;
		}

		// Прочитать все датчики. Возвращает количество удачных чтений.
		uint8_t ReadAll()
		{
			uint8_t ok = 0;
			for(uint8_t i = 0; i < _count; ++i)
			{
				if(_sensors[i].Read() == true)
					ok++;
			}
			_ReadyNotify();

			return ok;
		}


		// ---- Сводные значения по исправным датчикам (NO_VALID_TEMP, если исправных нет) ----

		int16_t GetMinTemp() const
		{
			int16_t result = NO_VALID_TEMP;
			for(uint8_t i = 0; i < _count; ++i)
			{
				if(_sensors[i].IsValid() == false) continue;
				if(result == NO_VALID_TEMP || _sensors[i].GetTemp() < result) result = _sensors[i].GetTemp();
			}

			return result;
		}

		int16_t GetAvgTemp() const
		{
			int32_t sum = 0;
			uint8_t num = 0;
			for(uint8_t i = 0; i < _count; ++i)
			{
				if(_sensors[i].IsValid() == false) continue;
				sum += _sensors[i].GetTemp();
				num++;
			}

			return (num > 0) ? (int16_t)(sum / num) : NO_VALID_TEMP;
		}

		int16_t GetMaxTemp() const
		{
			int16_t result = NO_VALID_TEMP;
			for(uint8_t i = 0; i < _count; ++i)
			{
				if(_sensors[i].IsValid() == false) continue;
				if(result == NO_VALID_TEMP || _sensors[i].GetTemp() > result) result = _sensors[i].GetTemp();
			}

			return result;
		}

	private:

		void _SearchBegin()
		{
			_count = 0;
			_driver.SearchBegin();

			return;
		}

		// Найти одно следующее устройство и добавить, если это датчик. false — поиск закончен.
		bool _SearchStep()
		{
			if(_count >= _max_sensor)
				return false;

			OneWireDriver::rom_t rom;
			if(_driver.SearchNext(rom) == false)
				return false;

			if(OneWireTSensor::IsSupported(rom) == true)
				_sensors[_count++] = OneWireTSensor(_driver, rom);

			return true;
		}


		void _ReadyNotify()
		{
			if(_callback_ready != nullptr)
				_callback_ready(_sensors, _count);

			return;
		}


		enum state_t : uint8_t
		{
			STATE_SEARCH_BEGIN,		// Начать поиск
			STATE_SEARCH,			// Идёт поиск
			STATE_NO_SENSOR,		// Датчики не найдены, ждём перед повтором
			STATE_CONVERT,			// Запустить конвертацию
			STATE_CONVERT_WAIT,		// Ждём окончания конвертации
			STATE_READ,				// Читаем датчики по одному
			STATE_PAUSE,			// Ждём следующего периода
		};

		OneWireDriver &_driver;
		OneWireTSensor _sensors[_max_sensor];
		uint8_t _count;
		callback_ready_t _callback_ready;

		uint16_t _period;
		state_t _state;
		uint8_t _idx;				// Индекс читаемого датчика, используется только в STATE_READ
		uint32_t _timer;
};
