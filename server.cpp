#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <bitset>
#include <queue>

#include <ctime>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/vfs.h>
#include <sys/stat.h>
#include <dirent.h>
#include "header.h"

bool run = true;


bool data_updated_16 = false;
bool data_need_send_16 = false;

bool data_updated = false;

std::ofstream out_data_file;
std::time_t curr_time;

std::mutex adc_mutex;
std::mutex data_mutex;

uint64_t total_pulses = 0;
uint32_t curr_pulses = 0;

bool adc_run = false;
bool file_rec_run = false;
bool allow_file_write = false;

uint16_t rec_per_file = 1200;
uint16_t data_average = 20;

uint16_t sync_source = SYNC_EXT;
uint16_t adc_timeout = 500;

uint16_t curr_file_record = 0;

std::string ch1_name("ch1_name");
std::string ch2_name("ch2_name");

void* map_base = (void *)-1;

std::queue <int> que_for_send;
uint32_t data[DATA_ARRAY_SIZE * 2];
uint16_t adc_data[DATA_ARRAY_SIZE * 2];

uint32_t data_for_file[DATA_ARRAY_SIZE * 2];
uint32_t data_for_send[DATA_ARRAY_SIZE * 2];


struct config
{
	uint16_t adc_timeout;
	uint16_t sync_source;
	uint16_t data_average;
	uint16_t rec_per_file;
	uint32_t curr_time;
};

struct state
{
	bool adc_run;
	bool file_rec_run;
	uint16_t reserverd;
	uint32_t curr_pulses;
	uint64_t total_pulses;
};

/*

----------------------------------------------------------------------------------------------------
	Prototypes.
----------------------------------------------------------------------------------------------------

*/

// read and write values to memory map
uint32_t read_value(uint32_t a_addr);
void write_value(uint32_t a_addr, uint32_t a_value);

// save all configuration parameters
void SaveAllParameters();

// load all configuration parameters
void LoadAllParameters();

void SetSyncSourceBit(uint16_t value);

// get data from adc once
int GetData(uint16_t *adc_data);

// make clock in mask bit of GPIO_PL_0 reg
void make_clock(uint32_t mask);

// parse command function (used for console)
std::vector <std::string> ParseCommand(char buf[]);


/*

----------------------------------------------------------------------------------------------------
	Save one parameter to SETUP_FILE_NAME via address
----------------------------------------------------------------------------------------------------

*/

template <typename T>
void SaveParameter(T value, int address)
{
	// open settings file with binary mode, also file opens for read/write access to exclude data flushing
	std::fstream file_wr(SETUP_FILE_NAME, std::fstream::binary | std::fstream::out | std::fstream::in);

	// if file exist
	if(file_wr.is_open())
	{
		// move to value address
		file_wr.seekp(address, file_wr.beg);

		// write value
		file_wr.write((char*)&value, sizeof(value));

		// close file
		file_wr.close();
	}
}

/*

----------------------------------------------------------------------------------------------------
	Write value to address of memory map.
----------------------------------------------------------------------------------------------------

*/

void write_value(uint32_t a_addr, uint32_t a_value)
{
	void* virt_addr = (void*)((uint8_t*)map_base + a_addr);
	*((uint32_t*) virt_addr) = a_value;
}


/*

----------------------------------------------------------------------------------------------------
	Read 32 bit  value from memory map.
----------------------------------------------------------------------------------------------------

*/

uint32_t read_value(uint32_t a_addr)
{
	void* virt_addr = (void*)((uint8_t*)map_base + a_addr);
	return *((uint32_t*) virt_addr);
}

/*

----------------------------------------------------------------------------------------------------
	Read 16 bit value from memory map.
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
			if( GetData(adc_data) > 0)
			{
				// copy data to golbal variable
				for(int i = 0; i < DATA_ARRAY_SIZE * 2; i++)
					data[i] += (uint32_t) adc_data[i];
				data_updated_16 = true;

				// total pulses
				total_pulses++;
				curr_pulses++;

				if(curr_pulses % data_average == 0)
				{
					data_mutex.lock();
					for(int i = 0; i < DATA_ARRAY_SIZE * 2; i++)
					{
						data_for_file[i] = data[i];
						data_for_send[i] = data[i];
					}
					data_mutex.unlock();

					// mark data was updated
					data_updated = true;
					allow_file_write = true;

					memset(data, 0, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));

				}

				// if file recording started and data collected
				if(file_rec_run && allow_file_write)
				{
					allow_file_write = false;

					// we need create new file
					if(curr_file_record == 0)
					{
						// making file name from current time
						curr_time = time(NULL);
						struct tm *tm_struct;
						tm_struct = localtime(&curr_time);

						// creating string stream for time formatting
						std::stringstream ss;
						ss << tm_struct->tm_year + 1900
							<< tm_struct->tm_mon + 1
							<< tm_struct->tm_mday << "_"
							<< tm_struct->tm_hour
							<< tm_struct->tm_min
							<< tm_struct->tm_sec << "_"
							<< ch1_name << "_"
							<< ch2_name << ".ch2";

						// convert stream to string
						std::string out_file_name = DATA_FILE_PATH + ss.str();

						// trying open/create stream file
						out_data_file.open(out_file_name);

						// if file wasn't opened
						if(!out_data_file.is_open())
						{
							file_rec_run = false;
							std::cout << "File open error" << std::endl;
						}
						else
						{
							out_data_file.write((char*)&rec_per_file, sizeof(uint16_t));	// #0x00 file records(plan)
							out_data_file.write((char*)&data_average, sizeof(uint16_t));	// #0x02 pulses average
							out_data_file.write((char*)&rec_per_file, sizeof(uint16_t));	// #0x04	file records(real)
							out_data_file.write((char*)&curr_time, sizeof(std::time_t));	// #0x06 time start
							out_data_file.write((char*)&curr_time, sizeof(std::time_t));	// #0x0a	time end

							// write ch1 name to data file header
							uint16_t name_len = 0;
							name_len = ch1_name.length() + 1;
							out_data_file.write((char*)&name_len, sizeof(uint16_t));

							char* name_buf = new char[name_len];
							std::strcpy(name_buf, ch1_name.c_str());
							out_data_file.write(	name_buf, name_len );
							delete[] name_buf;

							// write ch2 name to data file header
							name_len = ch2_name.length() + 1;
							out_data_file.write((char*)&name_len, sizeof(uint16_t));

							name_buf = new char[name_len];
							std::strcpy(name_buf, ch2_name.c_str());
							out_data_file.write( name_buf, name_len );
							delete[] name_buf;
						}
					}

					// if data file exists and opened
					if(out_data_file.is_open())
					{
						// write current file record number(for control)
						out_data_file.write((char*)&curr_file_record, sizeof(uint16_t));

						// write adc data array
						out_data_file.write((char*)data_for_file, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t));  // record

						curr_file_record++;
					}

					// if enough file records and file is open
					if(curr_file_record >= rec_per_file && out_data_file.is_open())
					{
						// end time recording to file header
						curr_time = time(NULL);
						out_data_file.seekp(0x0a, out_data_file.beg);
						out_data_file.write((char*)&curr_time, sizeof(std::time_t));

						// close file
						out_data_file.close();

						// flush file records counter
						curr_file_record = 0;
					}
				}
			}
			else //if( GetData(adc_data) <= 0)
				std::cout << "no sync." << std::endl;
		}
		else //!adc_run
		{
			// if data file is open
			if(out_data_file.is_open())
			{
				// end time writing to file header
				curr_time = time(NULL);
				out_data_file.seekp(0x0a, out_data_file.beg);
				out_data_file.write((char*)&curr_time, sizeof(std::time_t));

				// writing to file header real collected file records
				out_data_file.seekp(0x04, out_data_file.beg);
				out_data_file.write((char*)&curr_file_record, sizeof(uint16_t));

				// close file
				out_data_file.close();

				// flush record counter
				curr_file_record = 0;
			}

			// wait sysytem idle time
			std::this_thread::sleep_for(std::chrono::milliseconds(SYS_IDLE));
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
		if(sync_source == SYNC_INT)
		{
			// make clock for internal synchro
			make_clock(SYNC_INT_BIT);

			// wait internal synchro idle time (50ms typically)
			std::this_thread::sleep_for(std::chrono::milliseconds(SYNC_INT_DELAY));
		}
		else
			// wait system idle
			std::this_thread::sleep_for(std::chrono::milliseconds(SYS_IDLE));
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

	// initializing timeout calculation variables
	int tout_counter = 0;
	auto start_tick = std::chrono::high_resolution_clock::now();
	auto idle_tick = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(idle_tick - start_tick).count();

	// waiting adc1 and adc2 buffers are filled

	while( (adc1_ram_counter < ADC_READY_RAM_SIZE) | (adc2_ram_counter < ADC_READY_RAM_SIZE))
	{
		// read adc RAM filling counter
		adc1_ram_counter = read_value(ADC1_CADDR) - ADC1_RAM;
		adc2_ram_counter = read_value(ADC2_CADDR) - ADC2_RAM;

		// 10 microseconds - reaction time
		std::this_thread::sleep_for(std::chrono::microseconds(10));

		// getting elapse time
		idle_tick = std::chrono::high_resolution_clock::now();
		elapsed = std::chrono::duration_cast<std::chrono::microseconds>(idle_tick - start_tick).count();

		// if timeout.
		if( elapsed >= adc_timeout*1000 )
			break;
	}

	// Turn OFF ADC1 and ADC2 converting
	make_clock( ADC1_OFF | ADC2_OFF );

	// if timeout was
	if(elapsed >= adc_timeout*1000)
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
	Thread for sending data to connected peer.
----------------------------------------------------------------------------------------------------

*/

void ClientSendThread(int peer, bool *connected)
{
	//bool client_connected = true;
	int bytes_read = 0;

	// while peer connected
	while(*connected)
	{
		// if data need send (32 bit)
		if( !que_for_send.empty() && data_updated)
		{
			que_for_send.pop();
			data_mutex.lock();
			bytes_read = send(peer, data_for_send, DATA_ARRAY_SIZE * 2 * sizeof(uint32_t), 0);
			data_mutex.unlock();
			data_updated = false;
		}

		// if data need send (16 bit)
		if(data_need_send_16 && data_updated_16)
		{
			bytes_read = send(peer, adc_data, DATA_ARRAY_SIZE * 2 * sizeof(uint16_t), 0);
			data_updated_16 = false;
			data_need_send_16 = false;
		}

		//std::cout << data_updated << " " << que_for_send.size() << std::endl;

		// system idle
		std::this_thread::sleep_for(std::chrono::milliseconds(SYS_IDLE));
	}
}

/*

----------------------------------------------------------------------------------------------------
	Thread for getting commands from connected peer.
----------------------------------------------------------------------------------------------------

*/

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
					SaveParameter(total_pulses, TOTAL_PULSES_ADDR);
					break;

				// adc run
				case CMD_ADC_RUN:
					if(!adc_run)
					{
						curr_pulses = 0;
						curr_file_record = 0;
						allow_file_write = false;
						data_updated = false;
						data_updated_16 = false;
						adc_run = true;
					}

					break;

				// cmd set "register" which got in argument
				case CMD_SET:
					switch(argument)
					{
						// file record start/stop
						case REG_FILE_RUN:
							file_rec_run = (unsigned int)value;
							SaveParameter(file_rec_run, FILE_REC_RUN_ADDR);
							std::cout << "file rec changed to " << file_rec_run << std::endl;
							break;

						// modify 'sync_source' variable
						case REG_SYNC:
						{
							// setting up type of synchro got from peer
							sync_source = (unsigned int)value;

							SetSyncSourceBit(sync_source);

							SaveParameter(sync_source, SYNC_SOURCE_ADDR);
							break;
						}
						// modify 'adc_tomeout' variable
						case REG_TIMEOUT:
							adc_timeout = (unsigned int)value;
							SaveParameter(adc_timeout, ADC_TIMEOUT_ADDR);
							std::cout << "timeout set to: " << adc_timeout << std::endl;
							break;

						// records per file
						case REG_RECORDS:
							rec_per_file = (unsigned int) value;
							SaveParameter(rec_per_file, REC_PER_FILE_ADDR);
							std::cout << "records per file set to: " << rec_per_file << std::endl;
							break;

						// pulses average
						case REG_AVERAGE:
							data_average = (unsigned int) value;
							SaveParameter(data_average, DATA_AVERAGE_ADDR);
							std::cout << "data average set to: " << data_average << std::endl;
							break;

						// setting up system time is got from host
						case REG_TIME:
						{
							struct timeval *time_in_sec;
							time_in_sec = (struct timeval*)(&buffer[3]);
							std::cout << "sec.: " << time_in_sec->tv_sec << " usec.: " << time_in_sec->tv_usec << std::endl;
							// setting system time
							if(settimeofday(time_in_sec, NULL) < 0)
								std::cout << "time setting problem: -- " << strerror(errno) << " -- " << std::endl;
							else
								std::cout << "time successful updated" << std::endl;
							break;
						}

						case REG_CH1_NAME:
						{
							char* name_str = new char[bytes_read - 2];
							memcpy(name_str, &buffer[3], bytes_read - 2);
							ch1_name = name_str;

							std::cout << "adc ch1 name changed to: " << ch1_name << std::endl;

							SaveAllParameters();
							break;
						}

						case REG_CH2_NAME:
						{
							char* name_str = new char[bytes_read - 2];
							memcpy(name_str, &buffer[3], (bytes_read - 2));
							ch2_name = name_str;
							std::cout << "adc ch2 name changed to:: " << ch2_name << std::endl;
							SaveAllParameters();
							break;
						}

						default:
							std::cout << "SET command argument wrong." << std::endl;
							break;
					}//end of switch(reg)
					break;

				//if command GET
				case CMD_GET:
					// trying to send adc data to peer

					switch(argument)
					{
						case REG_CH1_NAME:
						{	// send adc channel 1 name to peer
							bytes_read = send(peer, ch1_name.c_str(), ch1_name.length() , 0);
							break;
						}

						case REG_CH2_NAME:
							// send adc channel 2 name to peer
							bytes_read = send(peer, ch2_name.c_str(), ch2_name.length(), 0);
							break;

						case REG_ADC_DATA:
							//Send only if adc is running and data is 'fresh'
							if( adc_run )
							   que_for_send.push(1);
							break;
						case REG_ADC_DATA_16:
							//Send only if adc is running and data is 'fresh'
							if( adc_run & data_updated_16 )
							   data_need_send_16 = true;
							break;
						case REG_COUNTER:
							// trying to send value of pulses counter to peer
							bytes_read = send(peer, &curr_pulses, sizeof(unsigned long), 0);
							break;

						case REG_CFG:
						{	// get all server registers
							config send_cfg;

							// filling config strucure
							send_cfg.adc_timeout = (uint16_t) adc_timeout;
							send_cfg.sync_source = sync_source;
							send_cfg.data_average = data_average;
							send_cfg.rec_per_file = rec_per_file;
							send_cfg.curr_time = (uint32_t) time(NULL);

							// send structure to peer
							bytes_read = send(peer, &send_cfg, sizeof(send_cfg),0);
							break;
						}

						// send to peer state structure
						case REG_STATE:
						{
							// fill sate structure
							state send_state;
							send_state.adc_run = adc_run;
							send_state.file_rec_run = file_rec_run;
							send_state.curr_pulses = curr_pulses;
							send_state.total_pulses = total_pulses;

							// send state structure
							bytes_read = send(peer, &send_state, sizeof(state),0);
							break;
						}
						default:
							std::cout << "Bad argument for GET command." << std::endl;
					}
					// case CMD_GET.
					break;

			default:
					// if command bad
					std::cout << "bad command." << std::endl;
					break;
			}// end switch(command)
		}//end if(data_read < 0)
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
/*
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
}*/


int check_files(std::string directory)
{
   DIR *dir;
   dirent *entry;
   dir = opendir(directory.c_str());

   int file_count = 0;
   struct stat st;

   if(dir)
   {
      while((entry = readdir(dir)) != nullptr)
      {
         if( (strcmp(entry->d_name,".") != 0) && (strcmp(entry->d_name, "..") != 0) )
         {
            std::string tmp_path = directory + entry->d_name;
            stat(tmp_path.c_str(), &st);

            if(S_ISREG(st.st_mode))
            {
               file_count++;
               std::cout << std::setw(15) << entry->d_name << "|";
               std::cout << std::setw(8) << st.st_size << "|";
               std::cout << std::setw(30) << tmp_path.c_str() << std::endl;
            }

            if(S_ISDIR(st.st_mode))
               file_count += check_files(tmp_path + "/");
         }
      }
   }

   return file_count;
}




/*

----------------------------------------------------------------------------------------------------
	Main function;
----------------------------------------------------------------------------------------------------

*/

int main()
{
	int f_device = -1;

	std::string dirname = "/mnt/data/";
	struct statfs fs;
	statfs(dirname.c_str(), &fs);

	std::cout << " available: " << ((fs.f_bsize/1024) * ((fs.f_blocks - fs.f_bfree + fs.f_bavail)/1024))/1024 << "GB; ";
	std::cout << " free: " << ((fs.f_bsize/1024) * (fs.f_bavail/1024))/1024 <<"GB; " << std::endl << std::endl;

	DIR *dir;
	dirent *entry;
	dir = opendir(dirname.c_str());

	int file_counter = 0;
	std::cout << std::setw(15) << "name" << std::setw(8) << "size" << std::setw(30) << "path" << std::endl;
	file_counter = check_files(dirname);

	std::cout << "total fles: " << file_counter << std::endl << std::endl;

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

	//load server parameters from binary config file
	LoadAllParameters();

	// accept syn_source
	SetSyncSourceBit(sync_source);

	// create tcp listener thread
	std::thread tcp_thr(ServerListener);
	tcp_thr.detach();

	// create console input thread
	//std::thread console_thr(ConsoleThread);
	//console_thr.detach();

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
		std::this_thread::sleep_for(std::chrono::milliseconds(SYS_IDLE));
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

	adc_mutex.lock();
	// make 'GPIO_PL_0' pulse like 'CLEAR->SET->CLEAR' bit added with mask
	write_value( GPIO_PL_0, state & (~mask) );
	write_value( GPIO_PL_0, state | mask );
	adc_mutex.unlock();
	std::this_thread::sleep_for(std::chrono::microseconds(SYNC_PULSE_WIDTH));
	adc_mutex.lock();
	write_value( GPIO_PL_0, state & (~mask) );
	adc_mutex.unlock();
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
	Save all setup parameters to settings file
----------------------------------------------------------------------------------------------------

*/

void SaveAllParameters()
{
	// file opening
	std::ofstream file_wr(SETUP_FILE_NAME, std::ofstream::binary);

	if(file_wr.is_open())
	{
		// serial writing parameters to settings file
		file_wr.write((char*)&adc_run, sizeof(bool));
		file_wr.write((char*)&file_rec_run, sizeof(bool));
		file_wr.write((char*)&rec_per_file, sizeof(uint16_t));
		file_wr.write((char*)&data_average, sizeof(uint16_t));
		file_wr.write((char*)&sync_source, sizeof(uint16_t));
		file_wr.write((char*)&adc_timeout, sizeof(uint16_t));
		file_wr.write((char*)&total_pulses, sizeof(uint64_t));

		// write ch1 name to settings file
		uint16_t name_len = 0;
		name_len = ch1_name.length() + 1;

		file_wr.write((char*)&name_len, sizeof(uint16_t));

		char* name_buf = new char[name_len];
		std::strcpy(name_buf, ch1_name.c_str());
		file_wr.write(	name_buf, name_len);

		delete[] name_buf;

		// write ch2 name to settings file
		name_len = ch2_name.length() + 1;
		file_wr.write((char*)&name_len, sizeof(uint16_t));

		name_buf = new char[name_len];
		std::strcpy(name_buf, ch2_name.c_str());
		file_wr.write( name_buf, name_len );

		delete[] name_buf;

		// closing file
		file_wr.close();
	}
}

/*

----------------------------------------------------------------------------------------------------
	read all setup parameters from settings file
----------------------------------------------------------------------------------------------------

*/


void LoadAllParameters()
{
	std::ifstream file_rd;
	file_rd.open(SETUP_FILE_NAME, std::ifstream::binary);

	if(file_rd.is_open())
	{
		// serial reading of parameters
		file_rd.read((char*)&adc_run, sizeof(bool));
		file_rd.read((char*)&file_rec_run, sizeof(bool));
		file_rd.read((char*)&rec_per_file, sizeof(uint16_t));
		file_rd.read((char*)&data_average, sizeof(uint16_t));
		file_rd.read((char*)&sync_source, sizeof(uint16_t));
		file_rd.read((char*)&adc_timeout, sizeof(uint16_t));
		file_rd.read((char*)&total_pulses, sizeof(uint64_t));

		// length of channel name
		uint16_t name_len;

		// reading ch1 name length from file
		file_rd.read((char*)&name_len, sizeof(uint16_t));

		// reading ch1 name from file with length
		char* name_buf = new char[(name_len)];
		file_rd.read(name_buf, name_len);

		//name_buf[name_len] = 0;
		ch1_name = name_buf;
		delete[] name_buf;

		// reading ch2 
		file_rd.read((char*) &name_len, sizeof(uint16_t));

		name_buf = new char[name_len + 1];
		file_rd.read(name_buf, name_len);

		ch2_name = name_buf;
		delete[] name_buf;

		// close config file
		file_rd.close();
	}
	else
	{
		SaveAllParameters();
	}
}

/*

----------------------------------------------------------------------------------------------------
	Set bit of synchronization source to hardware
----------------------------------------------------------------------------------------------------

*/

void SetSyncSourceBit(uint16_t value)
{
	// read state of GPIO_PL_0 register
	uint32_t read_data = read_value(GPIO_PL_0);

	// if sync. source internal - setup hrdware
	if(value == SYNC_INT)
	{
		write_value( GPIO_PL_0, read_data | (uint32_t) SYNC_SOURCE_BIT );
		std::cout << "sync source set to internal" << std::endl;
	}

	// if sync. source external - setup hardware
	if(value == SYNC_EXT)
	{
		write_value( GPIO_PL_0, read_data & ( ~((uint32_t)SYNC_SOURCE_BIT) ) );
		std::cout << "sync source set to external" << std::endl;
	}
}


/*

----------------------------------------------------------------------------------------------------
	End program;
----------------------------------------------------------------------------------------------------

*/
