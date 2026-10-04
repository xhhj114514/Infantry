#ifndef CHASSIS_TEST_MESSAGE_CENTER_H
#define CHASSIS_TEST_MESSAGE_CENTER_H

#include <stdint.h>

typedef struct { unsigned int unused; } Publisher_t;
typedef struct { unsigned int unused; } Subscriber_t;

Publisher_t *PubRegister(char *name, uint8_t data_len);
uint8_t PubPushMessage(Publisher_t *publisher, void *data);
Subscriber_t *SubRegister(char *name, uint8_t data_len);
uint8_t SubGetMessage(Subscriber_t *subscriber, void *data);

#endif
