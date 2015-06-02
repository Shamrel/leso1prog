/**
 \file leso1prog.c
 \author Shauerman Alexander <shamrel@yandex.ru>  www.labfor.ru
 \brief Загрузка hex-файла в память микроконтроллера учебного стенда LESO1
 \details Программа предназначена для загрузки исполняемого кода в память
 микропроцессора ADuC842 на платформе учебного лабораторного стенда LESO1.
 Программа взаимодействует со стендом через виртуальный последовательный
 порт ttyUSB (или другой). Программа имеет консольный интерфейс.

 \version   0.1
 \date 14.11.2014
 \copyright
leso1prog распространяется под лицензией BSD 2-ух пунктов. Эта лицензия дает
все права на использование и распространение программы в двоичном виде или
в виде исходного кода, при условии, что в исходном коде сохранится указание
авторских прав.

leso1prog is licensed under the simplified BSD license. This license gives
everyone the right to use and distribute the code, either in binary or
source code format, as long as the copyright license is retained in
the source code.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <err.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#define PROGNAME "leso1prog"
#define VERSION "v0.1"
/**
 \brief  Включение/выключение подробного вывода.
 \details
 #define DEBUG 1 -- печатаются сообщения PRINTF(...);
 #define DEBUG 0 -- сообщения PRINTF(...) игнорируются
 */
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#define ACK (0x06)
#define NAK (0x07)


/**
 \brief  Перевод стенда LESO1  в режим программирования.
 */
#define LESO1_PROG() setRTS(globalArgs.sport_fd,1)
/**
 \brief  Перевод стенда LESO1  в режим работы.
 */
#define LESO1_RUN() setRTS(globalArgs.sport_fd,0)
/**
 \brief  Сброс LESO1.
 */
#define LESO1_RESET() 						\
{		setDTR(globalArgs.sport_fd, 1); 	\
		usleep(30000);						\
		setDTR(globalArgs.sport_fd, 0);}


/**
 \struct hexfile_str
 \brief Структура хранит параметры строки hex-файла, согласно "INTEL HEX FORMAT".
 */
typedef struct hexfile_str {

	uint8_t len; 					//!< Длина записи.
	uint16_t load_addr;				//!< Начальный адрес записи.
	uint8_t type;					//!< Тип строки: 0 -- данные для записи; 1 -- конец файла.
	uint8_t data[0x40];				//!< Данные для записи.
	uint8_t crc;					//!< Контрольная сумма.
} hexfile_str_t;

/**
 \struct globalArgs
 \brief Глобальные переменные.
 */
struct globalArgs_t {
	const char *prog;			//!< Имя программы.
	char *siodevName;			//!< параметр -s. Имя последовательного порта.
	int sport_fd;				//!< Дескриптор для последовательного порта.
	char *hexFileName;			//!< параметр -x. Имя hex-файла.
	FILE *hex_fd;				//!< Дескриптор для hex-файла.
	uint8_t onlyErase;			//!< параметр -e. -- только очищаем память.
	uint8_t onlyCheckHex;		//!< параметр -с. -- только проверяем hex-файл
	uint8_t onlyReset;			//!< параметр -r. -- только сбрасываем микроконтроллер
} globalArgs;



void exit_handler(char *errmsg, char *msg )
{
	if(errmsg != NULL) fprintf(stderr,"\033[1;31mERROR:\033[0m %s\n",errmsg);
	if(msg != NULL) fprintf(stderr,"%s\n",msg);

	if(globalArgs.sport_fd > 0)
	{
		LESO1_RUN();
		LESO1_RESET();
		close(globalArgs.sport_fd);
	}
	if(globalArgs. hex_fd != NULL) fclose(globalArgs. hex_fd);
	if(errmsg != NULL) exit(EXIT_FAILURE);
		else exit(EXIT_SUCCESS);
}

void term_handler(int i)
{
	printf ("\n\033[32mClose %s\033[0m\n", globalArgs.prog);
	exit_handler(NULL, NULL);
}

// прототипы функций
int parse_hex_string (int8_t * str, hexfile_str_t *param);
int createWPack(uint8_t *pack, hexfile_str_t *param);
int createErasePack(uint8_t *pack, uint8_t tipe);
int interrogate(uint8_t *pack);
int check_hex_file(FILE *file);
uint8_t checksum(uint8_t *data);
int setRTS(int sport_fd, int State);
int setDTR(int sport_fd, int State);

#define usage(prog)	{printf("\n\033[1musage:\033[0m %s [-c|e|r] [-s siodev] [-x file.hex]\n", prog);}

static const char *optString = "Vechs:x:r";

int main(int argc, char **argv)
{
	uint8_t rx_buff[64], tx_buff[64], string_buff[256];

	struct termios tty_options;
	int16_t n, i;
	int16_t ret; 						// вспомогательная переменная, хранит код возврата функций.
	struct timeval timeout;
	fd_set         input;
	hexfile_str_t param;
	int16_t total_num_string;
	int c;
										// инициализирум структуру глобальных переменных
	globalArgs.prog = argv[0];
	globalArgs.siodevName = NULL;
	globalArgs.sport_fd = -1;
	globalArgs.hex_fd = NULL;
	globalArgs.onlyErase = 0;
	globalArgs.onlyCheckHex = 0;
	globalArgs.onlyReset = 0;

	if(argc == 1) 						// мало параметров
	{
		usage(globalArgs.prog);
		exit(EXIT_FAILURE);
	}
	opterr = 0;
	while((c = getopt(argc, argv, optString)) != -1)
	{
		switch(c)
	    {
	    case 's':
	       	if(optarg == NULL)
	       	{
	       		usage(globalArgs.prog);
	       		exit_handler("Unknown siodev\n", NULL);
	       	}
	    	globalArgs.siodevName = optarg;
	    	if (access(globalArgs.siodevName, W_OK|R_OK|W_OK) != 0) 	// siodev не существует
    		{
	    		fprintf(stderr,"\033[1;31mERROR:\033[0m serial device \033[1m%s\033[0m doesn't exist or access denied\n", globalArgs.siodevName);
	    		exit(EXIT_FAILURE);
    		};
	    	break;
	    case 'x':
	    	if(optarg == NULL)
			{
				usage(globalArgs.prog);
				exit_handler("Unknown hex-file\n", NULL);
			}
	    	globalArgs.hexFileName = optarg;
			if (access(globalArgs.hexFileName, R_OK|W_OK) != 0) 	// hex-файл не существует
			{
				fprintf(stderr,"\033[1;31mERROR:\033[0m file \033[1m%s\033[0m doesn't exist or access denied\n", globalArgs.hexFileName);
				exit(EXIT_FAILURE);
			};
			break;

	    case 'e':
	    	globalArgs.onlyErase = 1;
	    	break;

	    case 'r':
	   	    	globalArgs.onlyReset = 1;
	   	    	break;

	    case 'c':
	    	globalArgs.onlyCheckHex = 1;
	    	break;

	    case 'V':
	    	printf("%s %s\n",PROGNAME, VERSION);
	    	exit_handler(NULL,NULL);
	    	break;
	    case '?':
	    case 'h':
	    default:
	    	usage(globalArgs.prog);
	    	fprintf(stderr,"\033[1mOptions are:\033[0m\n");
	    	fprintf(stderr," -s siodev      Serial device (e.g. /dev/ttyUSB0)\n");
	    	fprintf(stderr," -x file.hex    Intel Hex standard file\n");
	    	fprintf(stderr," -e             Perform a chip erase\n");
			fprintf(stderr," -r             Perform a chip reset\n");
	    	fprintf(stderr," -c             Perform check the hex-file\n");
	    	fprintf(stderr," -h             Print this information\n");
	    	fprintf(stderr," -V             Print version information and exit\n");
	    	exit(EXIT_FAILURE);
	    	break;
	    }
	}

	/* Настраиваем сигналы */

	struct sigaction sa;
	sa.sa_handler = term_handler;
	sigaction(SIGINT, &sa, 0);

	if ((globalArgs.hexFileName != NULL))				// если имя файла существует
	{
		globalArgs.hex_fd = fopen(globalArgs.hexFileName,"r");						// открываем hex файл. только для чтения
		if (globalArgs.hex_fd == NULL) exit_handler("open hex-file",NULL);
		total_num_string = check_hex_file(globalArgs.hex_fd);
		if (total_num_string<=0) exit_handler("bad hex-filename!",NULL);
		if (globalArgs.onlyCheckHex)
		{

			printf("\n rows = %i\n\033[1m%s\033[0m is valid!\n\033[0m",total_num_string, globalArgs.hexFileName);
			fclose(globalArgs.hex_fd);
			exit(EXIT_SUCCESS);						// -c
		}

	}else if (globalArgs.onlyCheckHex)
		exit_handler("hex-filename not specified!",
			"\033[1musage:\033[0m -v -x file.hex\n");

	// к этому моменту мы завершили все действия, возможные без последовательного порта

	if (globalArgs.siodevName != NULL )
		globalArgs.sport_fd = open (globalArgs.siodevName, O_RDWR | O_NOCTTY | O_NONBLOCK);
//		globalArgs.sport_fd = open (globalArgs.siodevName, O_RDWR | O_NOCTTY );
		else exit_handler("bad serial device name", NULL);
	if (globalArgs.sport_fd < 0)
	{
		perror("open");
		exit_handler("open serial", NULL);
	}

	// Настраиваем порт
	PRINTF("Настраиваем порт\n");
	if(tcgetattr(globalArgs.sport_fd, &tty_options) == -1) perror("tcgetattr: "); 	// считываем текущие параметры
	cfsetispeed(&tty_options, B9600); 												// скорость на прием
	cfsetospeed(&tty_options, B9600); 												// скорость на передачу
	//tty_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOCTL | ECHOKE);
	tty_options.c_iflag = 0;
	tty_options.c_lflag = 0;
	//tty_options.c_cflag = 0;
	tty_options.c_oflag = 0;

//	printf("c_lflag = 0x%x\n",tty_options.c_lflag );
//	printf("c_cflag = 0x%x\n",tty_options.c_cflag );
//	exit_handler(NULL,NULL);

	tty_options.c_cflag &= ~CSIZE; 													// Маскирование битов размера символов
	tty_options.c_cflag &= ~PARENB;
	tty_options.c_cflag &= ~CSTOPB;
	tty_options.c_cflag |= (CLOCAL | CREAD | CS8); 									//Разрешение приемника и установка локального режима, установка 8 битов данных

	if(tcsetattr(globalArgs.sport_fd, TCSAFLUSH, &tty_options) == -1) perror("tcsetattr"); // устанавливаем параметры

	if(tcflush(globalArgs.sport_fd, TCIOFLUSH) == -1) perror("tcflush: "); 			// очищаем очередь ввода/вывода

	if(globalArgs.onlyReset)	// -r
	{
		LESO1_RUN();
		LESO1_RESET();
		exit_handler(NULL, "\033[1mADuC reset\033[0m");
	}
	// переходим к загрузке hex-файла

	// Мы должны запустить бутлоадер только если требуется стереть память,
	// то есть установлена опция "-v" или мы собираемся прошивать ее,
	// а для этого должен быть успешно открыт и проверен hex-файл.
	// Если файл УЖЕ открыт, то он УЖЕ проверен, иначе программа бы завершилась.
	if((globalArgs.onlyErase == 0)&&(globalArgs.hex_fd == NULL))
		exit_handler("Unknown hex-file", NULL);

	FD_ZERO(&input);		// Настраиваем набор дескрипторов для работы select
	FD_SET(globalArgs.sport_fd, &input);

	int16_t ptr = 0;

	memset(rx_buff, 0, sizeof(rx_buff));
	LESO1_PROG();			// Запуск бутлоадера
	LESO1_RESET();
	LESO1_RUN();
	while(ptr < 25)			// Принимаем 14 байт описания
	{
		timeout.tv_sec =1;
		timeout.tv_usec = 0;
		ret = select(globalArgs.sport_fd+1, &input, NULL, NULL, &timeout);
		if (ret < 0) { perror("select failed"); exit_handler("system error", NULL);}
		else if (ret == 0) exit_handler("ADuC doesn't respond", "error starting bootloader");
		else if ((ret = read(globalArgs.sport_fd, &rx_buff[ptr], 25)) > 0)
				ptr += ret;
	}
	rx_buff[14]='\0';					// после 14-го байта идут непечатные символы
	printf("Loader reply : %s \n", rx_buff);

	memset(rx_buff, 0, sizeof(rx_buff));
	ret = createErasePack(tx_buff, 1);		// создаем пакет для стирания памяти
#if DEBUG
	printf("Create erasy tx-package:\n");
	for(i=0; i < ret ; i++) printf("0x%02hhx ",tx_buff[i]); printf("\n");
#endif
	write(globalArgs.sport_fd, tx_buff, ret );

	timeout.tv_sec =5;
	timeout.tv_usec = 0;

	ret = select(globalArgs.sport_fd+1, &input, NULL, NULL, &timeout);	// ждем отклик
	if (ret < 0) { perror("select failed"); exit_handler("system error", NULL);}
	else if (ret == 0) exit_handler("ADuC doesn't respond", "error erase chip");
	else if ((ret = read(globalArgs.sport_fd, &rx_buff[0], 25)) > 0)
		{
			PRINTF("resp = %#02hhx\r\n", rx_buff[0]);
			if (rx_buff[0] == ACK) printf("Program flash memory cleared\n");
			else exit_handler("memory erasing FAILED \n",NULL);
		}
	if(globalArgs.onlyErase) exit_handler(NULL,NULL);	// если требовалась только стереть память, то завершаем программу

	if(globalArgs.hex_fd == NULL) exit_handler("Unknown hex-file", "NULL");
	uint16_t str_count = 0;
	fseek(globalArgs.hex_fd, 0, SEEK_SET);								// устанавливаем указатель на начало файла
	printf("\n");
	while (fgets (string_buff, sizeof(string_buff) , globalArgs.hex_fd) != NULL)
	{																	// считываем строки пока файл не закончится
		if (parse_hex_string(string_buff, &param))						// поочередно работаем с каждой строкой
		{															    // парсим строку, выделяем адреса, явки, пароли.
			exit_handler("bad hex-file", NULL);							// в файле ошибка, продложать смысла нет.
		}
		if (param.type) // последня строка hex-файла, заврешаем прошивку
			break;

		ret = createWPack(tx_buff, &param);

#if DEBUG
	printf("Str %u, pack to send[%u]:\r\n", str_count, ret);
	for(i=0; i < ret ; i++) printf("%02hhx ",tx_buff[i]); printf("\n");
#endif

	str_count++;
#if !DEBUG
		//Анимация прогресса
		char c;
		switch (3&(str_count>>3)){
		case 0: c = '|'; break;
		case 1: c = '\\'; break;
		case 2: c = '|'; break;
		case 3: c = '/'; break;
		default: c = '*'; break;
		}

		//printf("%c%u%%\033[1F\n",c, (str_count*100/total_num_string));

		printf("\033[1F%c%u%%\n",c, (str_count*100/total_num_string));
#endif

		write(globalArgs.sport_fd,tx_buff,ret );						// посылаем пакет микроконтроллеру
		timeout.tv_sec =0;
		timeout.tv_usec = 100000;										// таймаут 100 мс.
		ret = select(globalArgs.sport_fd+1, &input, NULL, NULL, &timeout);	// ждем отклик
		if (ret < 0) { perror("select failed"); exit_handler("system error", NULL);}
		else if (ret == 0) exit_handler("ADuC doesn't respond", "error download hex");
		else if ((ret = read(globalArgs.sport_fd, &rx_buff[0], 1)) > 0)
		{
			#if DEBUG
				printf("resp = %#02hhx\r\n", rx_buff[0]);
			#endif
			if (rx_buff[0] == NAK)  exit_handler("Hex-string rejected by ADuC\n",NULL);
		}
	}

	LESO1_RUN();		// запускаем контроллер
	LESO1_RESET();
	exit_handler(NULL,"\033[1F100%\n\033[1mFlashing complete\033[0m\n");

	return 0;
}


/**
\brief Функция разбирает строку hex-файла на параметры.
\details Функция сканирует строку hex-файла, выделяет адрес записи,
число байт данных, проверяет контрольную сумму. Полученными результатами
функция заполняет структуру param.
\param str Указатель на строку для анализа
\param param Структура, содержащая параметры и данные строки. Выходные данные.
\throw -1 Строка не соответсвует INTEL HEX FORMAT.
\throw -2 Ошибка контрольной суммы.
*/

int parse_hex_string (int8_t * str, hexfile_str_t *param)
{
	uint8_t crc=0;								// Аккумулятор для контрольной суммы.
	if(str[0] != ':') return (-1); 				// это строка не из hex-файла.

	sscanf(&str[1], "%02hhx", &param->len);		// считываем число байт
	crc += param->len;

	sscanf(&str[3], "%04hx", &param->load_addr);// считываем начальный адрес
	crc += param->load_addr;
	crc += param->load_addr>>8;

	sscanf(&str[7], "%02hhx", &param->type);		// считываем тип строки
	crc += param->type;

	str += 9;									// переводим указательна начало данных.
												// непосредственно данные в строке начинаются с 9-го символа.
	int i;
	for (i = 0; i < param->len; i++)			// последовательно считываем все байты данных.
	{
		sscanf(str, "%02hhx", &param->data[i]);
		str += 2;
		crc += param->data[i];
	}
	sscanf(str, "%02hhx", &param->crc);			// считываем контрольную сумму
	//PRINTF("crc = 0x%02hhx   0x%02x\n",crc, param->crc);

	if ( 0xFF&(param->crc+crc) ) return (-2);	// если расчитанная сумма не совпала с записанной в конце строки,
												// то возвращаем ошибку
	return 0;
}

/**
\brief Функция формирует пакет для отправки микроконтроллеру.
\details Пакет содержит информацию для записи в память программ или данных.
Формируется пакет на основании параметров, полученных из одной строки hex-файла.
\param pack Указатель на буфер символов, в котором должен быть сформирован пакет.
\param param Структура, содержащая параметры и данные для передачи.
\return Число байт для отправки.
*/
int createWPack(uint8_t *pack, hexfile_str_t *param)
{
	uint8_t ptr = 0;
	pack[ptr++] = 0x07;									// магическое число
	pack[ptr++] = 0x0E;									// магическое число
	pack[ptr++] = param->len + 4;						// число байт данных
	pack[ptr++] = 'W';									// идентификатор пакета
	pack[ptr++] = 0;									// адрес страницы
	pack[ptr++] = (uint8_t)0xFF&(param->load_addr>>8);	// старший байт адреса
	pack[ptr++] = (uint8_t)0xFF&(param->load_addr);		// старший байт адреса

	int i;
	for (i = 0; i < param->len; i++)
	{
		pack[ptr++] = param->data[i];
	}
	pack[ptr++] = checksum(&pack[2]);
	return ptr;
};

/**
\brief Функция формирует пакет с командой для очистки памяти ADuC.
\param pack Указатель на буфер символов, в котором должен быть сформирован пакет.
\param tipe tipe == 0, чистим всю память;
			tipe == 1, чистим только память программ.
\return Число байт для отправки.
*/
int createErasePack(uint8_t *pack, uint8_t tipe )
{
	uint8_t ptr = 0;
	pack[ptr++] = 0x07;						// магическое число
	pack[ptr++] = 0x0E;						// магическое число
	pack[ptr++] = 1;						// число байт данных
	pack[ptr++] = tipe?'C':'A';				// идентификатор пакета
	pack[ptr++] = checksum(&pack[2]);
	return ptr;
}

/**
\brief Функция формирует пакет с командой для запроса информации об встроенном загрузчике.
\param pack Указатель на буфер символов, в котором должен быть сформирован пакет.
\return Число байт для отправки.
*/
int interrogate(uint8_t *pack )
{
	uint8_t ptr = 0;
	pack[ptr++] = '!';
	pack[ptr++] = 'Z';
	pack[ptr++] = 0;
	pack[ptr++] = 0xA6;
	return ptr;
}
/**
\brief Функция проверяет hex-файл на целостность и соответствие формату.
\details Функция сканирует каждую строку файла,
выделяет адрес записи, число байт данных, проверяет
контрольную сумму
\param file -- указатель на поток. (файловый дескриптор)
\return число строк в файле
\throw -1 хотя бы одна строка не соответсвует INTEL HEX FORMAT
\throw -2 нет завершающей строки
*/
int check_hex_file(FILE *file)
{

	uint8_t string_buff[256];											// буфер
	uint16_t str_count;
	hexfile_str_t param;
	fseek(file, 0, SEEK_SET);											// устанавливаем указатель на начало файла
	while (fgets (string_buff, sizeof(string_buff) , file) != NULL)		// считываем строки пока файл не закончится
	{																	// поочередно работаем с каждой строкой
		if (parse_hex_string(string_buff, &param))						// парсим строку
			return (-1);												// в файле ошибка, продложать смысла нет
		str_count++;
		if (param.type) 												// последня строка hex-файла, заврешаем проверку
			return str_count;											// возвращаем число строк
	}
	return (-2);														// ошибка. В файле нет завершающей строки
};

uint8_t checksum(uint8_t *data)
{
	uint8_t i, crc = 0;
	for (i=0;i <= data[0]; i++ )
	{
		crc -= data[i];
//	printf("0x%02x  0x%02x \n",data[i], crc);
	}
	return crc;
}

int setRTS(int sport_fd,int State)
{
    int status; // current status

      if (ioctl (sport_fd, TIOCMGET, &status) == -1)
      {
            perror("TIOCMGET failed");
            return -1;
      }
    if (State)  // set bit
    {
        status |= TIOCM_RTS;
        PRINTF("Set RTS 1\n");
        if (ioctl (sport_fd, TIOCMSET, &status) == -1)
        {
            perror("TIOCMSET failed ");
            return -1;
        }
    }
    else // clear bit
    {
        status &= ~TIOCM_RTS;
        PRINTF("Set RTS 0\n");
        if (ioctl (sport_fd, TIOCMSET, &status) == -1)
        {
            perror("TIOCMSET failed ");
            return -1;
        }
    }
    return 0;
}

int setDTR(int sport_fd,int State)
{
	int status; // current status

	if (ioctl (sport_fd, TIOCMGET, &status) == -1)
	{
		perror("TIOCMGET failed ");
		return -1;
	}
	if (State)  // set bit
	{
		status |= TIOCM_DTR;
		PRINTF("Set DTR 1\n");
		if (ioctl (sport_fd, TIOCMSET, &status) == -1)
		{
			perror("TIOCMSET failed ");
			return -1;
		}
	}
	else // clear bit
	{
		status &= ~TIOCM_DTR;
		PRINTF("Set DTR 0\n");
		if (ioctl (sport_fd, TIOCMSET, &status) == -1)
		{
			perror("TIOCMSET failed ");
			return -1;
		}
	}
	return 0;
}
