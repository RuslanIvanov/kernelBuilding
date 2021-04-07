
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

#ifndef __IOCTL_PVV3_H__
#define __IOCTL_PVV3_H__

#define PVV3_IOC_MAGIC 0xfa //как системный номер

#define PVV3_IOCRESET_DPU	_IO(PVV3_IOC_MAGIC,    0)
#define PVV3_IOCG_IRQ_DPU  	_IOR(PVV3_IOC_MAGIC,   1,  int)

#define PVV3_IOC_MAXNR 2

#define PVV3_BUF 512

#endif
