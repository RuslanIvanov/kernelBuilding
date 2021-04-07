
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

#ifndef __IOCTL_PVV4_H__
#define __IOCTL_PVV4_H__

#define PVV4_IOC_MAGIC 0xfb //как системный номер

#define PVV4_IOCRESET_DPU_RM	_IO(PVV4_IOC_MAGIC,    0)
#define PVV4_IOCG_IRQ_DPU_RM  	_IOR(PVV4_IOC_MAGIC,   1,  int)

#define PVV4_IOC_MAXNR 2

#define PVV4_BUF 512

#endif
