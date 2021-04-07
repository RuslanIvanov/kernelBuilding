
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

#ifndef __IOCTL_PVV2_H__
#define __IOCTL_PVV2_H__

#define PVV2_IOC_MAGIC 0xfe //как системный номер

#define PVV2_IOCRESET_RM      	_IO(PVV2_IOC_MAGIC,    0)
#define PVV2_IOCRESET_DPU	_IO(PVV2_IOC_MAGIC,    1)
#define PVV2_IOCG_IRQ_RM  	_IOR(PVV2_IOC_MAGIC,   2,  int)
#define PVV2_IOCG_IRQ_DPU  	_IOR(PVV2_IOC_MAGIC,   3,  int)
#define PVV2_IOCG_IRQ_WD	_IOR(PVV2_IOC_MAGIC,   4,  int)

#define PVV2_IOC_MAXNR 5
#define PVV2_BUF 512

#endif
