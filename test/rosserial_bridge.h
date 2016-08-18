#include <stdio.h>
#include "stm32f4xx.h"
#include <rtthread.h>uint16_t topic_id;

typedef struct
{
	uint16_t topic_id;
	const char* topic_name;
	const char* message_type;
	const char* md5sum;
	int32_t buffer_size;
	
}TopicInfo;

enum { 	ID_PUBLISHER = 0 ,
		ID_SUBSCRIBER = 1,
		ID_SERVICE_SERVER = 2,
		ID_SERVICE_CLIENT = 4,
		ID_PARAMETER_REQUEST = 6,
		ID_LOG = 7 ,
		ID_TIME = 10,
		ID_TX_STOP = 11 };

const char * getType(){ return "rosserial_msgs/TopicInfo"; };
const char * getMD5(){ return "0ad51f88fc44892f8c10684077646005"; };