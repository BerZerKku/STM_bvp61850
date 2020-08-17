#ifndef FIFO_H_
#define FIFO_H_

#include "stdint.h"
#include "stdio.h"

template <size_t len, typename type>
class TFifo {
	type fifo[len];

public:
	/// �����������.
	TFifo() {
		head = 0;
		size = 0;
	}

	/**	�������� �������� � �������.
	 *
	 * 	@param[in] var ��������.
	 * 	@return False - ���� ������� ���������, ����� true.
	 */
	bool push(type var) {
	  bool isovf = true;

		if (size < len) {
			fifo[(head + size++) % len] = var;
			isovf = false;
		}

		return !isovf;
	}

	/**	������� ���������� �� �������.
	 *
	 *	@param[out] ��������.
	 *	@return False - ���� ������� �����, ����� true.
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

	///	������� �������.
	void flush() {
		size =0;
	}

public:
	/// ������ �������.
	size_t head;

	/// ����� �������.
	size_t size;
};



#endif /* FIFO_H_ */
