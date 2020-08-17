#ifndef FIFO_H_
#define FIFO_H_

#include "stdint.h"
#include "stdio.h"

template <size_t len, typename type>
class TFifo {
	type fifo[len];

public:
	/// Конструктор.
	TFifo() {
		head = 0;
		size = 0;
	}

	/**	Добавить значение в очередь.
	 *
	 * 	@param[in] var Значение.
	 * 	@return False - если очередь заполнена, иначе true.
	 */
	bool push(type var) {
	  bool isovf = true;

		if (size < len) {
			fifo[(head + size++) % len] = var;
			isovf = false;
		}

		return !isovf;
	}

	/**	Достать переменную из очереди.
	 *
	 *	@param[out] Значение.
	 *	@return False - если очередь пуста, иначе true.
	 */
	bool pop(type &var) {
	  bool isempty = true;

		if (size > 0) {
			var = fifo[head];
			head = (head + 1) % len;
			size--;
			isempty = false;
		}

		return !isempty;
	}

	///	Очистка очереди.
	void flush() {
		size =0;
	}

public:
	/// Начало очереди.
	size_t head;

	/// Конец очереди.
	size_t size;
};



#endif /* FIFO_H_ */
