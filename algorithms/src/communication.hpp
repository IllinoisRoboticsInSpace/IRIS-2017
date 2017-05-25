
struct locate_motor{
int motor_right;
int motor_left;
};

locate_motor get_desired_motor();

struct tcp_send
{
	char* address;
	int port;
	int sock; //Socket for tcp client

	tcp_send(char* _address, int _port);
	int send(char* data, int size);
	int receive(char* data, int size);

private: 
	void internal_reconnect();
};

void* communication_motor(void * unused);
void* communication_actuators(void * unused);