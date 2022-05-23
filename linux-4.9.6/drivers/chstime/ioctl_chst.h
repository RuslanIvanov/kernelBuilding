
//команды ввода вывода

/*
_IO(type,nr) - для команды, которая не имеет аргумента;
_IOR(type,nr,datatype) - для чтения данных из драйвера;
_IOW(type,nr,datatype) - для записи данных;
_IOWR(type,nr,datatype) - для двунаправленной передачи. 
Поля type и fields, передаваемые в качестве аргументов, и поле
size получаются применением sizeof к аргументу datatype
*/

/*
* S означает "Set" ("Установить") через ptr,
* T означает "Tell" ("Сообщить") прямо с помощью значения аргумента
* G означает "Get" ("Получить"): ответ устанавливается через указатель
* Q означает "Query" ("Запрос"): ответом является возвращаемое значение
* X означает "eXchange" ("Обменять"): переключать G и S автоматически
* H означает "sHift" ("Переключить"): переключать T и Q автоматически
*/

#ifndef __IOCTL_KSDT_H__
#define __IOCTL_KSDT_H__

#define KSDT_IOC_MAGIC 0xfd //как системный номер

#define KSDT_IOCRESET	 	_IO(KSDT_IOC_MAGIC,    0)
#define KSDT_IOCG_PERIOD  	_IOR(KSDT_IOC_MAGIC,   1,  int)
#define KSDT_IOCS_PERIOD	_IOW(KSDT_IOC_MAGIC,   2,  int)

#define KSDT_IOC_MAXNR 3

#endif
