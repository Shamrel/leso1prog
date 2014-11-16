leso1prog
=========

LESO1 ADuC842 Downloader
Загрузка hex-файла в память микроконтроллера учебного стенда LESO1
Программа предназначена для загрузки исполняемого кода в память
микропроцессора ADuC842 на платформе учебного лабораторного стенда LESO1.
Программа взаимодействует со стендом через виртуальный последовательный
порт ttyUSB (или другой). Программа имеет консольный интерфейс.

Установка:
1. из архива:
	tar -xvzf leso1prog-v0.1.tar.gz
	cd leso1prog-v0.1/
	make

2. из git-репозитория:
	git clone git@github.com:Shamrel/leso1prog.git
	cd leso1prog-v0.1/
	make

usage: ./leso1prog [-c] [-e] [-s siodev] [-x file.hex]
Options are:
 -s siodev      Serial device (e.g. /dev/ttyUSB0)
 -x file.hex    Intel Hex standard file
 -e             Perform a chip erase
 -c             Perform check the hex-file
 -h             Print this information
 -V             Print version information and exit

Примеры использования программы:
1. Загрузить hex-файл в память микроконтроллера. Через параметр -s указываем 
путь к файлу устройства, через которое подключен стенд, например /dev/ttyUSB0; 
через параметр -x указываем путь к hex-файлу.
	./leso1prog -s /dev/ttyUSB0 -x main.hex
2. Только стереть память микроконтроллера и выйти. Указываем путь к устройству 
и параметр -e (от английского erase – стирать):
	./leso1prog -s /dev/ttyUSB0 -e
3. Только проверить hex-файл. Указываем путь к файлу и параметр -с 
(от английского check – проверять.):
	./leso1prog -x main.hex -c
4. Только сбросить микроконтроллер. Указываем путь к устройству и 
параметр -r (от reset):
	./leso1prog -s /dev/ttyUSB0 -r
5. Вывести версию leso1prog и выйти (-V  – от Version):
	./leso1prog -V
6. Напечатать справку и выйти (-h  – от help):
	./leso1prog -h
	
При загрузке hex-файла программой автоматически осуществляется проверка файла, а также 
стирание памяти, поэтому указывать дополнительно параметры -c -e не нужно. В некоторых 
случаях для доступа к последовательному устройству /dev/ttyUSB могут понадобится 
права суперпользователя.