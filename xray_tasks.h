#ifndef XRAY_TASKS_H
#define XRAY_TASKS_H

/* TODO move into external header */
#define N_PORTS 2
/* TODO move into external header */
#define ADC_RESULT_LEN 2

/* TODO move into appropriate header. */
/** Card read data representation. */
struct CardRead_t {
	uint8_t Port;
	uint8_t TimeTicks;
	uint8_t length;
	uint8_t PacketType;
	uint32_t CardNumber;
};

/** Tasks that are able to send data to USART. */
enum {
	/** REX/DPS processor task identifier. */
	AB_PROCESSOR_TASK_ID,
	/** Card read processor task identifier. */
	CARD_READ_PROCESSOR_TASK_ID,
	/** Inbound USART command processor task identifier. */
	COMMAND_PROCESSOR_TASK_ID,
	/** Input voltage threshold reporter task identifier. */
	VOLTAGE_THRESHOLD_REPORTER_TASK_ID,
	/** Low overcurrent reporter task identifier. */
	LOW_OVERCURRENT_REPORTER_TASK_ID,
	/** High overcurrent reporter task identifier. */
	HIGH_OVERCURRENT_REPORTER_TASK_ID,
	/** Number of UsartMessageBuffers elements.
	 * @attention Must always be the last element of the enumeration. */
	USART_BUFFERS_COUNT
};

/** USART message sizes. */
enum {
	AB_MESSAGE_LEN = 3,
	CARD_READ_MESSAGE_LEN = 11,
	COMMAND_PROCESSOR_IN_MESSAGE_MAX = 51,
	COMMAND_PROCESSOR_OUT_MESSAGE_MAX = 51,
	OVERCURRENT_MESSAGE_MAX = 7,
	VOLTAGE_THRESHOLD_MESSAGE_MAX = 7,
};

/** Buffer capacities */
enum {
	AB_BUFFER_LEN = 2 * N_PORTS,
	CARD_READ_BUFFER_LEN = 2 * N_PORTS * ( sizeof(struct CardRead_t) + sizeof(size_t) ),
	INPUT_VOLTAGE_AVERAGER_RAW_LEN = 4 * (ADC_RESULT_LEN + sizeof(size_t)),
	PERIPHERAL_CURRENT_AVERAGER_RAW_LEN = 4 * (ADC_RESULT_LEN + sizeof(size_t)),
	USART_INPUT_BUFFER_LEN = 80,
};

/** Timer periods, ms. */
enum {
	CARD_READ_TIMEOUT = 40,
	THRESHOLD_CHECKER_TIMEOUT = 100,
	LOW_OVERCURRENT_TIMEOUT = 3000,
	HIGH_OVERCURRENT_TIMEOUT = 1000,
	GAUGE_TIMER_TIMEOUT = 1000
};

extern QueueHandle_t ResponseQueue;
extern MessageBufferHandle_t *UsartMessageBuffers;

#endif /* XRAY_TASKS_H */
