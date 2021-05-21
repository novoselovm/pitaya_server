#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>

#include <ctime>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>

#define BASE_ADDR		0x40000000
#define GPIO_PL_0		(0x40020000 - BASE_ADDR)

#define ADC1_RAM		0x40000000
#define ADC2_RAM		0x40010000

#define ADC1_GPIO		0x40040000
#define ADC2_GPIO		0x40030000

#define ADC1_DMA		0x40050000
#define ADC2_DMA		0x40060000

#define ADC1_TMR		0x40050014
#define ADC2_TMR		0x40060014

#define ADC1_CTRL		(0x40050018 - BASE_ADDR)
#define ADC2_CTRL		(0x40060018 - BASE_ADDR)

#define ADC1_CADDR	(0x40050020 - BASE_ADDR)
#define ADC2_CADDR	(0x40060020 - BASE_ADDR)

#define ADC1_ON		0x100
#define ADC2_ON		0x400

#define ADC1_OFF		0x200
#define ADC2_OFF		0x800

#define ADC_RAM_SIZE	 		0x8000
#define ADC_READY_RAM_SIZE	0x7E00
#define DATA_BUF_SIZE 		0x4000

#define DATA_ARRAY_SIZE			0x4000	// adc 16384 of 16 bit elements per channel

#define DATA_FILE_PATH 		"/mnt/data/myfile/"

#define MAP_SIZE 		0x70000
#define MAP_MASK 		(MAP_SIZE - 1)


#define DEV_ID			0xFA

#define CMD_EXIT 		0xFF
#define CMD_SET		0x02
#define CMD_GET		0x04
#define CMD_ADC_RUN	0x08
#define CMD_ADC_STOP 0x0a

#define REG_SYNC			0x02
#define REG_TIME			0x04
#define REG_ADC_DATA		0x06
#define REG_ADC_DATA_16	0x07
#define REG_TIMEOUT		0x08
#define REG_COUNTER		0x09
#define REG_FILE_RUN 	0x0a

#define SYNC_EXT			0xFF
#define SYNC_INT			0x01
#define SYNC_SOURCE_BIT 0x02
#define SYNC_INT_BIT		0x01
#define SYNC_INT_DELAY	50000

bool run = true;
bool adc_run = false;

bool data_updated_16 = false;
bool data_need_send_16 = false;

bool data_updated = false;
bool data_need_send = false;

bool file_rec_run = false;

bool allow_file_write = false;

std::ofstream out_data_file;
std::time_t curr_time;

unsigned long pulses_counter = 0;

unsigned int rec_per_file = 1200;

unsigned int data_average = 20;

unsigned int sync_source = SYNC_EXT;
unsigned int adc_timeout = 500;

unsigned int curr_file_record = 0;
unsigned int file_size = 0;

void* map_base = (void *)-1;
uint32_t data[DATA_ARRAY_SIZE * 2];
uint16_t adc_data[DATA_ARRAY_SIZE * 2];

uint32_t data_for_file[DATA_ARRAY_SIZE * 2];
uint32_t data_for_send[DATA_ARRAY_SIZE * 2];





/*

----------------------------------------------------------------------------------------------------
	Prototypes.
----------------------------------------------------------------------------------------------------

*/

// read and write values to memory map
uint32_t read_value(uint32_t a_addr);
void write_value(uint32_t a_addr, uint32_t a_value);

// get data from adc once
int GetData(uint16_t *adc_data);

// make clock in mask bit of GPIO_PL_0 reg
void make_clock(uint32_t mask);

// parse command function (used for console)
std::vector <std::string> ParseCommand(char buf[]);
/*

----------------------------------------------------------------------------------------------------
	Write value to address.
----------------------------------------------------------------------------------------------------

*/

void write_value(uint32_t a_addr, uint32_t a_value)
{
	void* virt_addr = (void*)((uint8_t*)map_base + a_addr);
	*((uint32_t*) virt_addr) = a_value;
}


/*

----------------------------------------------------------------------------------------------------
	Read 32 bit  value from reg.
----------------------------------------------------------------------------------------------------

*/

uint32_t read_value(uint32_t a_addr)
{
	void* virt_addr = (void*)((uint8_t*)map_base + a_addr);
	return *((uint32_t*) virt_addr);
}

/*

----------------------------------------------------------------------------------------------------
	Read 16 bit value from reg.
----------------------------------------------------------------------------------------------------

*/

uint16_t read_value_16(uint32_t a_addr)
{
	void* virt_addr = (void*)((uint8_t*)map_base + a_addr);
	return *((uint16_t*) virt_addr);
}

/*

----------------------------------------------------------------------------------------------------
	Thread for gathering data from adc
----------------------------------------------------------------------------------------------------

*/

void adcThread()
{
	uint32_t read_data;

	// flush array of data memories
	memset(data, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));
	memset(data_for_file, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));
	memset(data_for_send, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));
	memset(adc_data, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint16_t));

	// while server running
	while(run)
	{
		if(adc_run)// adc runs
		{
			// if data got success

			int count = GetData(adc_data);
			if(count > 0)
			{
				// copy data to golbal variable
				for(int i = 0; i < DATA_ARRAY_SIZE * 2; i++)
					data[i] += (uint32_t) adc_data[i];
				data_updated_16 = true;

				// total pulses
				pulses_counter++;

				if(pulses_counter % data_average == 0)
				{
					// mark data updated event

					//data_need_send = true;

					for(int i = 0; i < DATA_ARRAY_SIZE * 2; i++)
					{
						data_for_file[i] = data[i];
						data_for_send[i] = data[i];
					}

					data_updated = true;
					allow_file_write = true;

					memset(data, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));
				}

				if(file_rec_run && allow_file_write)
				{
					allow_file_write = false;

					if(curr_file_record == 0)
					{
						curr_time = time(NULL);

						struct tm *tm_struct;
						tm_struct = localtime(&curr_time);

						// creating string stream for time formatting
						std::stringstream ss;
						ss << tm_struct->tm_year + 1900 << "-"
							<< tm_struct->tm_mon + 1 << "-"
							<< tm_struct->tm_mday << " "
							<< tm_struct->tm_hour << "-"
							<< tm_struct->tm_min << "-"
							<< tm_struct->tm_sec << ".ch2";

						// convert stream to string
						std::string time_str = DATA_FILE_PATH + ss.str();

						// trying open/create stream file
						out_data_file.open(time_str);

						// if file wasn't opened
						if(!out_data_file)
						{
							file_rec_run = false;
							std::cout << "File open error" << std::endl;
						}
						else
						{
							out_data_file.write((char*)&rec_per_file, sizeof(uint32_t));	// #0x00 file records(plan)
							out_data_file.write((char*)&data_average, sizeof(uint32_t));	// #0x04 pulses average
							out_data_file.write((char*)&rec_per_file, sizeof(uint32_t));	// #0x08	file records(real)
							out_data_file.write((char*)&curr_time, sizeof(std::time_t));	// #0x0c time start
							out_data_file.write((char*)&curr_time, sizeof(std::time_t));	// #0x10	time end
						}
					}

					if(out_data_file.is_open())
					{
						out_data_file.write((char*)&curr_file_record, sizeof(uint32_t));				// number of record
						out_data_file.write((char*)data, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));  // record

						curr_file_record++;
					}

					if(curr_file_record >= 1200)
					{
						curr_time = time(NULL);
						out_data_file.seekp(0x10, out_data_file.beg);
						out_data_file.write((char*)&curr_time, sizeof(std::time_t));
						out_data_file.close();
						curr_file_record = 0;
					}
				}
			}
			else
				std::cout << "no sync." << std::endl;
		}
		else
		{
			if(out_data_file.is_open())
			{
				curr_time = time(NULL);
				out_data_file.seekp(0x10, out_data_file.beg);
				out_data_file.write((char*)&curr_time, sizeof(std::time_t));
				out_data_file.seekp(0x08, out_data_file.beg);
				out_data_file.write((char*)&curr_file_record, sizeof(unsigned int));
				out_data_file.write((char*)&curr_time, sizeof(std::time_t));
				out_data_file.close();
			}
			usleep(1000); // TODO: replace usleep with proc. control
		}
	}
}

/*

----------------------------------------------------------------------------------------------------
	Thread for internal synchro programmical generator.  deault is 50ms (20 Hz)
----------------------------------------------------------------------------------------------------

*/

void IntSyncThread()
{
	while(run)
	{
		// if adc is running and sync source is internal
		if( adc_run && sync_source == SYNC_INT)
		{
			make_clock(SYNC_INT_BIT);
			usleep(SYNC_INT_DELAY);
		}
		else
			usleep(1000);
		// TODO: replace usleep with thread control functions
	}
}

/*

----------------------------------------------------------------------------------------------------
	Get data from adc.
----------------------------------------------------------------------------------------------------

*/

int GetData(uint16_t *adc_data)
{
	// Turn on ADC1 and ADC2 converting
	make_clock( ADC1_ON | ADC2_ON );

	// Waiting until AD 1 and 2 finish converting
	uint32_t adc1_ram_counter = 0;
	uint32_t adc2_ram_counter = 0;

	// waiting adc data ready with timeout
	int tout_counter = 0;

	// waiting adc1 and adc2 buffers are filled
	while((adc1_ram_counter < ADC_READY_RAM_SIZE) | (adc2_ram_counter < ADC_READY_RAM_SIZE))
	{
		// read adc RAM filling counter
		adc1_ram_counter = read_value(ADC1_CADDR) - ADC1_RAM;
		adc2_ram_counter  = read_value(ADC2_CADDR) - ADC2_RAM;

		usleep(10);

		// TODO: timeout with real time.
		tout_counter++;

		// if timeout.
		if(tout_counter >= adc_timeout*10)
			break;
	}

	// Turn OFF ADC1 and ADC2 converting
	make_clock( ADC1_OFF | ADC2_OFF );

	// if timeout was
	if(tout_counter >= adc_timeout*10)
		return -1;


	// TODO: may be optimize with pointers
	int idx = 0;
	for(int ram_addr = 0; ram_addr < ADC_RAM_SIZE; ram_addr += 2)
	{
		adc_data[idx] = read_value_16(ADC1_RAM - BASE_ADDR + ram_addr) & 0x3FFF;
		idx++;
	}

	for(int ram_addr = 0; ram_addr < ADC_RAM_SIZE; ram_addr += 2)
	{
		adc_data[idx] = read_value_16(ADC2_RAM - BASE_ADDR + ram_addr) & 0x3FFF;
		idx++;
	}

	return idx;
}

/*

----------------------------------------------------------------------------------------------------
	Thread for connected clients.
----------------------------------------------------------------------------------------------------

*/

void ClientSendThread(int peer, bool *connected)
{
	//bool client_connected = true;
	int bytes_read = 0;

	while(*connected)
	{
		if(data_need_send)
		{
			bytes_read = send(peer, data_for_send, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t), 0);
			data_need_send = false;
		}
		else
			usleep(1000);

		if(data_need_send_16)
		{
			bytes_read = send(peer, adc_data, DATA_ARRAY_SIZE * 2 * sizeof(uint16_t), 0);
			data_need_send_16 = false;
		}
		else
			usleep(1000);
	}
}

void ClientThread(int peer)
{
	char buffer[64];
	int bytes_read, pos, from, len;

	int dev_addr = 0;
	int command = 0;
	int argument = 0;
	int value = 0;
	bool client_connected = true;

	std::thread client_send_thr(ClientSendThread, peer, &client_connected);

	while(client_connected)
	{
		// flush recieve buffer
		for(int i = 0; i < 64; i++)
			buffer[i] = 0;

		// blocking call: waiting for client data
		bytes_read = recv(peer, buffer, 64, 0);

		// 'parsing' commad
		dev_addr = (int)buffer[0];
		command = (int)buffer[1];
		argument = (int)buffer[2];

		// parsing 16 bit value from buffer[3-4]
		value = 0x00 | (int)(buffer[3] << 8) | (int)(buffer[4]);

		// but if wasn't client query
		if(bytes_read <= 0)
		{
			std::cout << std::endl << "client disconnected;" << std::endl;
			client_connected = false;
		}
		else
		{
		// Command interpretation
		//-------------------------------------------------------

			switch(command)
			{
				// exit srver command
				case CMD_EXIT:
					run = false;
					client_connected = false;
					close(peer);
					break;

				// stop adc conmmand
				case CMD_ADC_STOP:
					adc_run = false;
					break;

				// adc run
				case CMD_ADC_RUN:
					adc_run = true;
					break;

				// cmd set "register" which got in argument
				case CMD_SET:

					// file record start/stop
					if(argument == REG_FILE_RUN)
					{
						file_rec_run = (unsigned int)value;
						std::cout << "file_rec: " << file_rec_run << std:: endl;
					}

					// modify 'sync_source' variable
					if(argument == REG_SYNC)
					{
						// TODO: SetSync function?
						// setting up type of synchro got from peer
						sync_source = (unsigned int)value;

						// read state of GPIO_PL_0 register
						uint32_t read_data = read_value(GPIO_PL_0);

						// if sync. source internal - setup hrdware
						if(sync_source == SYNC_INT)
							write_value( GPIO_PL_0, read_data | (uint32_t) SYNC_SOURCE_BIT );

						// if sync. source external - setup hardware
						if(sync_source == SYNC_EXT)
							write_value( GPIO_PL_0, read_data & ( ~((uint32_t)SYNC_SOURCE_BIT) ) );
					}

					// modify 'adc_tomeout' variable
					if(argument == REG_TIMEOUT)
					{
						adc_timeout = (unsigned int)value;
					}

					// TODO: time synchronization when peer connected
					// setting up system time is got from host
					if(argument == REG_TIME)
					{
						struct tm *ts;
						struct timeval time_in_sec;
						//struct timezone tz;
						double *fract_sec;

						//tz.tz_minuteswest = -420; //tomsk?
						//tz.tz_dsttime = 0;

						// gettting time structure and fractional seconds from buffer
						ts =  (struct tm*) (&buffer[11]);
						fract_sec = (double*)(&buffer[3]);

						//gettimeofday(&time_in_sec, &tz);

						//std::cout << "timezone " << tz.tz_minuteswest << " dst: " << tz.tz_dsttime << std::endl;

						// cast time to linux format
						ts->tm_year = ts->tm_year - 1900;
						ts->tm_mon = ts->tm_mon - 1;
						ts->tm_hour = ts->tm_hour;

						// fill timeval structure for settimeofdate
						time_in_sec.tv_sec = mktime(ts);
						time_in_sec.tv_usec = (unsigned long) 1000000.0 * (*fract_sec);

						// setting system time
						if(settimeofday(&time_in_sec, NULL) < 0)
							std::cout << "time setting problem: -- " << strerror(errno) << " -- " << std::endl;
					}
					break;

				//if command GET
				case CMD_GET:
					// trying to send adc data to peer
					if(argument == REG_ADC_DATA)
					{
						std::cout << "data_reg cmd" << std::endl;
						//Send only if adc is running and data is 'fresh'
						if( adc_run & data_updated )
							data_need_send = true;

						std::cout << "data_reg end" << std::endl;
					}

					if(argument == REG_ADC_DATA_16)
					{
						//Send only if adc is running and data is 'fresh'
						if( adc_run & data_updated_16 )
							data_need_send_16 = true;
					}

					// trying to send value of pulses counter to peer
					if( argument == REG_COUNTER)
					{
						bytes_read = send(peer, &pulses_counter, sizeof(unsigned long), 0);
					}
					break;

				default:
					// if command bad
					std::cout << "bad command." << std::endl;
					break;
			}// end switch(command)
		}
	}// end while(client_connected)

	// waiting for client thread ended 
	client_send_thr.join();
}//end of function

/*

----------------------------------------------------------------------------------------------------
	Server listener thread function.
----------------------------------------------------------------------------------------------------

*/

// TODO: may be need to save array of connected clients, and send it to peer by query
// and make class for clients with static methon for 'recieve <-> send'

void ServerListener()
{
	int listener, client;
	struct sockaddr_in client_addr;

	// address structure and its len
	struct sockaddr_in my_peer;
	socklen_t peer_size;

	// SO_REUSEADDR option and bind state
	int en_option, bind_state;
	bind_state = -1;
	en_option = 1;

	//  1) int domain: AF_INET - all IP nets; 2) int type: SOCK_STREAM - full duplex stream connection;
	// 3) int protocol: 0 - default protocol;
	listener = socket(AF_INET, SOCK_STREAM, 0);

	// if creating listener trouble
	if(listener < 0)
	{
		std::cout << "ERROR: listener creating error" << std::endl;
		run = 0;
		exit(0);
	}

	// fill address structure for binding socket
	client_addr.sin_family = AF_INET;
   client_addr.sin_port = htons(3113); 			// htons() - converts ushort to TCP/IP network byte order;
   client_addr.sin_addr.s_addr = htonl(INADDR_ANY);	// hnonl() - -/--/-/- ulong -/-/-/-/-/-//-

	// SO_REUSEADDR - allow use port connection immediate after closing connection
	setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,&en_option, sizeof(int));

   // binding server to port client_addr.sin_port
   bind_state =  bind(listener, (struct sockaddr *) &client_addr, sizeof(client_addr));

	// if was binding error
	if( bind_state  < 0 )
   {
      std::cout << std::endl << "ERROR: listener binding error: { " << strerror(errno) << " };" << std::endl;
      run = 0;
      exit(0);
   }

	// start listenig clients at port client_addr.sin_port with 5 connection limit
   if( listen(listener, 5) < 0)
   {
      std::cout << std::endl << "ERROR: listener starting error" << std::endl;
      run = 0;
      exit(0);
   }

	while(1)
	{
   	peer_size = sizeof(my_peer);
		// blocking call: waiting for client connection.
   	client = accept(listener, (struct sockaddr *) &my_peer, &peer_size);

		// if there is a problem
   	if(client < 0)
   	{
      	std::cout << "ERROR: { " << strerror(errno) << " };" << std::endl;
   	}
		else
		{
   		std::cout << "client connected: " << inet_ntoa(my_peer.sin_addr) << std::endl;
			// create thread for peer service
			std::thread client_thr(ClientThread, client);
			client_thr.detach();
		}
	}

	// closing sock desc.
	close(client);
   close(listener);

	// message for understanding the server was stoped correct
   std::cout << std::endl << "---- tcp server stoped ----" << std::endl << std::endl;
}

/*

----------------------------------------------------------------------------------------------------
	iostream thread function.
----------------------------------------------------------------------------------------------------

*/

void ConsoleThread()
{
	std::string incoming;

	// cicle waiting while main program  works
	while(run)
	{
		// input string
		std::cin >> incoming;

		std::vector <std::string> arr_command = ParseCommand((char*) incoming.c_str());

		// if exit command recieved
		if(arr_command[0] == "exit")
			run = 0;
	}
}


/*

----------------------------------------------------------------------------------------------------
	Main function;
----------------------------------------------------------------------------------------------------

*/

int main()
{
	int f_device = -1;

	// open PL device memory part like file
	f_device = open("/dev/mem", O_RDWR | O_SYNC); 

	// if fileopen fault
	if(f_device == -1)
	{
		std::cout << "Device /dev/mem open error." << std::endl;
		exit(0);
	}

	// viewing PL device memory file to OS RAM (memory mapping)
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, f_device, BASE_ADDR & ~MAP_MASK);

	// if memory mapping fault
	if(map_base == (void*) -1)
	{
		std::cout << "Device memory mapping error." << std::endl;
		exit(0);
	}

	// create tcp listener thread
	std::thread tcp_thr(ServerListener);
	tcp_thr.detach();

	// create console input thread
	std::thread console_thr(ConsoleThread);
	console_thr.detach();

	// adc idle thread
	std::thread adc_thread(adcThread);
	adc_thread.detach();

	// internal synchronization thread
	std::thread internal_sync_thr(IntSyncThread);
	internal_sync_thr.detach();

	// disconnect threads from main program
	// because they have blocking calls

	curr_time = time(NULL);

	// message to console for understanding server is started
	std::cout << std::endl << "---- Server started at " << ctime(&curr_time) << std::endl << std::endl;

	// non blocking wait threads done;
	while(run)
	{
		usleep(1000);
	}

	// message for understanding correct server shutdown
	std::cout << std::endl << " ----- server was stopped by user ----- " << std::endl;
	exit(0);
}



/*

----------------------------------------------------------------------------------------------------
	Function make clock pulse like _/\_ with mask;
----------------------------------------------------------------------------------------------------

*/

void make_clock(uint32_t mask)
{
	uint32_t state;
	// get initial GPIO_PL_0 state
	state = read_value(GPIO_PL_0);

	// make 'GPIO_PL_0' pulse like 'CLEAR->SET->CLEAR' bit added with mask
	write_value( GPIO_PL_0, state & (~mask) );
	write_value( GPIO_PL_0, state | mask );
	write_value( GPIO_PL_0, state & (~mask) );
}


/*

----------------------------------------------------------------------------------------------------
	Parsing command function. (used for console)
----------------------------------------------------------------------------------------------------

*/

std::vector <std::string> ParseCommand(char buf[])
{
	std::vector <std::string> v_command;
	std::string recieved = "";
	int space_count = 0;
	bool was_symbol = false;

	for(int i = 0; i<strlen(buf); i++)
	{
		if(buf[i] != ' ')
		{
			recieved += buf[i];
			was_symbol = true;
		}
		else if(was_symbol)
		{
			space_count++;

			if(space_count > 0 && recieved.length() > 0)
			{
				space_count = 0;
				v_command.push_back(recieved);
				recieved = "";
			}
		}
	}

	if(recieved.length() > 0)
		v_command.push_back(recieved);

	return v_command;
}

/*

----------------------------------------------------------------------------------------------------
	End program;
----------------------------------------------------------------------------------------------------

*/

