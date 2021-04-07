
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

#ifndef __IOCTL_PVV_H__
#define __IOCTL_PVV_H__

#define PVV_IOC_MAGIC 0xfc //как системный номер

#define PVV_IOCRESET	          _IO(PVV_IOC_MAGIC,    0)
#define PVV_IOCRESET_PARENT       _IO(PVV_IOC_MAGIC,    1)
#define PVV_IOCRESET_CHILD        _IO(PVV_IOC_MAGIC,    2)
#define PVV_IOCG_IRQ_PARENT   	  _IOR(PVV_IOC_MAGIC,   3,  int)
#define PVV_IOCG_IRQ_CHILD   	  _IOR(PVV_IOC_MAGIC,   4,  int)
#define PVV_IOCG_IRQ_CHILD_WD     _IOR(PVV_IOC_MAGIC,   5,  int)

#define PVV_IOC_MAXNR 6

#define PVV_BUF 512

#endif
