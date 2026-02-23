/*****************************************************************//**
 * \file   Queue.c
 * \brief  循环队列的实现，参考《数据结构》（严蔚敏）
 * 
 * \author LiBing
 * \date   July 2020
 *********************************************************************/ 
#include <stddef.h>
#include <string.h>
//自定义头文件
#include "mcTypes.h"

#include "Queue.h"
//队列操作通用函数
/**
 * \brief 循环队列的初始化.
 * 
 * \param Q			队列指针。
 * \param e			数据块的基地址(起始地址)。
 * \param MaxSize	数据空间最大数量（循环队列长度比数据空间小1）。
 * \return			返回零执行成功，非零执行失败。
 */
int InitQueue( Queue* Q, void* e,int MaxSize, int ElemSize) {//构造一个空队列Q
    WaitForSingleObject(Q->hMutex, INFINITE);   
    Q->lock=1;
	if (e == NULL) { goto ERR; }//起始地址为空,队列未初始化.
	if (MaxSize < 2) { goto ERR; }//数据空间小于2.
	Q->ElemSize = ElemSize;   //单个元素所占的存储空间，单位字节
	Q->MaxSize = MaxSize;	//数据空间为MaxSize，循环队列元素最大数量为MaxSize-1，最大元素下标为MaxSize-2.
	Q->FreeSize = MaxSize - 1;
	Q->base = e;
	Q->front = Q->rear = 0; //头指针和尾指针置为零，队列为空。
    ReleaseMutex(Q->hMutex);
 
	return 0;
ERR:
	ReleaseMutex(Q->hMutex);
	return 1;
}

/**
 * \brief 循环队列的初始化.
 *
 * \param Q			队列指针。
 * \param e			数据块的基地址(起始地址)。
 * \param MaxSize	数据空间最大数量（循环队列长度比数据空间小1）。
 * \return			返回零执行成功，非零执行失败。
 */
int InitQueueISR(Queue* Q, void* e, int MaxSize, int ElemSize) {//构造一个空队列Q
	Q->lock=1;
	if (e == NULL) { goto ERR; }//起始地址为空,队列未初始化.
	if (MaxSize < 2) { goto ERR; }//数据空间小于2.
	Q->ElemSize = ElemSize;   //单个元素所占的存储空间，单位字节
	Q->MaxSize = MaxSize;	//数据空间为MaxSize，循环队列元素最大数量为MaxSize-1，最大元素下标为MaxSize-2.
	Q->FreeSize = MaxSize - 1;
	Q->base = e;
	Q->front = Q->rear = 0; //头指针和尾指针置为零，队列为空。
	return 0;
ERR:
	return 1;
}



//队列操作通用函数
/**
 * \brief 循环队列的清空.
 *
 * \param Q			队列指针。
 * \param e			数据块的基地址(起始地址)。
 * \param MaxSize	数据空间最大数量（循环队列长度比数据空间小1）。
 * \return			返回零执行成功，非零执行失败。
 */
int ClearQueueISR(Queue* Q) {//构造一个空队列Q
	Q->FreeSize = Q->MaxSize - 1;
	Q->front = Q->rear = 0; //头指针和尾指针置为零，队列为空。
	return 0;
}

/**
 * \brief		求循环队列的长度.
 * 
 * \param Q		队列指针.
 * \return		队列长度.
 */
int QueueLength( Queue* Q) {//返回Q的元素个数，即队列的长度
	int len = 0;
	WaitForSingleObject(Q->hMutex, INFINITE);
	len=(Q->rear - Q->front + Q->MaxSize) % Q->MaxSize;
	ReleaseMutex(Q->hMutex);
	return len;
}

/**
 * \brief		（中断内使用）求循环队列的长度.
 * 
 * \param Q		队列指针.
 * \return		队列长度.
 */
int QueueLengthISR( Queue* Q) {//返回Q的元素个数，即队列的长度
	int len = 0;
	len=(Q->rear - Q->front + Q->MaxSize) % Q->MaxSize;
	return len;
}




/**
 * \brief		求循环队列剩余空间.
 *
 * \param Q		队列指针.
 * \return		队列长度.
 */
int QueueFreeSize( Queue* Q) {//返回Q的空闲空间
	int len = 0;
	WaitForSingleObject(Q->hMutex, INFINITE);
	len = Q->FreeSize;
	ReleaseMutex(Q->hMutex);
	return len;
}

/**
 * \brief		求循环队列剩余空间.
 *
 * \param Q		队列指针.
 * \return		队列长度.
 */
int QueueFreeSizeISR(Queue* Q) {//返回Q的空闲空间
	return Q->FreeSize;
}

/**
 * \brief 插入元素e为Q的新的队尾元素.
 * 
 * \param Q		队列指针。
 * \param e		插入的数据元素的指针。
 * \return		返回零执行成功，非零执行失败。
 */
int EnQueue( Queue* Q, void* e) {//插入元素e为Q的新的队尾元素
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char *)Q->base+ Q->rear* Q->ElemSize;
    if ((Q->rear + 1) % Q->MaxSize == Q->front) //尾指针在循环意义上加1后等于头指针，表明队满
		goto ERR;
	memcpy(addr, e, Q->ElemSize);
	Q->rear = (Q->rear + 1) % Q->MaxSize; //队尾指针加1
	Q->FreeSize = Q->FreeSize - 1;//队列剩余大小
	ReleaseMutex(Q->hMutex);
	return 0;
ERR:
	ReleaseMutex(Q->hMutex);
	return 1;
}


/**
 * \brief 插入元素e为Q的新的队尾元素.
 *
 * \param Q		队列指针。
 * \param e		插入的数据元素的指针。
 * \return		返回零执行成功，非零执行失败。
 */
int EnQueueISR(Queue* Q, void* e) {//插入元素e为Q的新的队尾元素
	char* addr = (char*)Q->base + Q->rear * Q->ElemSize;
	if ((Q->rear + 1) % Q->MaxSize == Q->front) //尾指针在循环意义上加1后等于头指针，表明队满
		goto ERR;
	memcpy(addr, e, Q->ElemSize);
	Q->rear = (Q->rear + 1) % Q->MaxSize; //队尾指针加1
	Q->FreeSize = Q->FreeSize - 1;//队列剩余大小
	return 0;
ERR:
	return 1;
}

/**
 * \brief 循环队列的出队,删除Q的队头元素，用e返回其值.
 * 
 * \param Q		队列指针。
 * \param e		返回对头元素的地址。
 * \return		返回值零执行成功，非零执行失败。
 */
int DeQueue( Queue* Q, void* e) {//删除Q的队头元素，用e返回其值
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char *)Q->base + Q->front * Q->ElemSize;
    if (Q->front == Q->rear)
		goto ERR; //队空
	memcpy(e, addr, Q->ElemSize);
	Q->front = (Q->front + 1) % Q->MaxSize; //队头指针加1
	Q->FreeSize = Q->FreeSize + 1;
	ReleaseMutex(Q->hMutex);
	return 0;
ERR:
	ReleaseMutex(Q->hMutex);
	return 1;
}

/**
 * \brief (没有互斥量，在中断内使用,或在任务中关闭中断后使用)循环队列的出队,删除Q的队头元素，用e返回其值.
 * 
 * \param Q		队列指针。
 * \param e		返回对头元素的地址。
 * \return		返回值零执行成功，非零执行失败。
 */
int DeQueueISR( Queue* Q, void* e) {//删除Q的队头元素，用e返回其值
	char* addr = (char *)Q->base + Q->front * Q->ElemSize;
   if (Q->front == Q->rear)
		goto ERR; //队空
	memcpy(e, addr, Q->ElemSize);
	Q->front = (Q->front + 1) % Q->MaxSize; //队头指针加1
	Q->FreeSize = Q->FreeSize + 1;
	return 0;
ERR:
	return 1;
}

//返回Q的队头元素，不修改队头指针
/**
 * \brief 返回Q的队头元素，不修改队头指针.
 * 
 * \param Q		队列指针。
 * \param e		返回对头元素的地址。
 * \return		返回零执行成功，非零表示队列为空。
 */
int GetHeadElemISR( Queue* Q, void* e) {//返回Q的队头元素，不修改队头指针
	char* addr = (char *)Q->base + Q->front * Q->ElemSize;
	if (Q->front == Q->rear)
		goto ERR; //队空	
	memcpy(e, addr, Q->ElemSize);
	return 0;
ERR:
	return 1;
}

//返回Q的队头元素，不修改队头指针
/**
 * \brief 返回Q的队头元素，不修改队头指针.
 * 
 * \param Q		队列指针。
 * \param e		返回对头元素的地址。
 * \return		返回零执行成功，非零表示队列为空。
 */
int GetHeadElem( Queue* Q, void* e) {//返回Q的队头元素，不修改队头指针
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char*)Q->base + Q->front * Q->ElemSize;
	if (Q->front == Q->rear)
		goto ERR; //队空
	memcpy(e, addr, Q->ElemSize);
	ReleaseMutex(Q->hMutex);
	return 0;
ERR:
	ReleaseMutex(Q->hMutex);
	return 1;
}

void* GetHeadPtr( Queue* Q) {//返回Q的队头地址，不修改队头指针
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char*)Q->base + Q->front * Q->ElemSize;
	if (Q->front == Q->rear)
		goto ERR; //队空
	ReleaseMutex(Q->hMutex);
	return addr;
ERR:
	ReleaseMutex(Q->hMutex);
	return NULL;
}

void* getElemFromRear( Queue* Q,int idx) {//从队尾取元素指针，不删除元素
	double len=(Q->rear - Q->front + Q->MaxSize) % Q->MaxSize;

	if (Q->front==Q->rear || idx + 1 >len)//从队尾数起，不能超过队头
		goto ERR;
	char* addr = (char*)Q->base + ((Q->rear-idx-1)%Q->MaxSize) * Q->ElemSize;
	return addr;
ERR:
	 
	return NULL;
}

void* getElemFromHead( Queue* Q,int idx) {//从队头取元素指针，不删除元素
	double len=(Q->rear - Q->front + Q->MaxSize) % Q->MaxSize;
	if (Q->front==Q->rear || idx + 1 >len)//从队尾数起，不能超过队头
		goto ERR;
	char* addr = (char*)Q->base + ((Q->front+idx)%Q->MaxSize) * Q->ElemSize;
	return addr;
ERR:
	 
	return NULL;
}

void* getPreviousElem(Queue* Q, int *idx ) {

	if (*idx  == Q->front //在队头位置了，表明前面没有元素了
 	||Q->front==Q->rear) //队空
		goto ERR;
	char* addr = (char*)Q->base + *idx * Q->ElemSize;
	//memcpy(e, addr, Q->ElemSize);
	*idx=(*idx - 1) % Q->MaxSize;
	return addr;
ERR:
	return NULL;
}

void *getCurrentElem(Queue* Q, int idx) {

	if(Q->front==Q->rear\
		|| idx==Q->rear) //队空
		goto ERR;
	char* addr = (char*)Q->base + idx * Q->ElemSize;
	//memcpy(e, addr, Q->ElemSize);
	return addr;
ERR:
	return NULL;

}


// void *getNextElem(Queue* Q, int *idx) {

// 	if (*idx   == Q->rear //表明到队尾了,没有元素了
// 	||Q->front==Q->rear) //队空
// 		goto ERR;
// 	char* addr = (char*)Q->base + *idx * Q->ElemSize;
// 	//memcpy(e, addr, Q->ElemSize);
// 	*idx = (*idx + 1) % Q->MaxSize; 
// 	return addr;
// ERR:
// 	return NULL;
// }

int isQueueRear(Queue* Q, int idx)
{
	if ((idx+1)%Q->MaxSize  == Q->rear) //表明在队尾
		return 1;
	else
	    return 0;
}

int isQueueHead(Queue* Q, int idx)
{
	if (idx  == Q->front) //表明在队头
		return 1;
	else
	    return 0;
}

int getNextIndex(Queue* Q, int idx)
{
	if ((idx + 1)%Q->MaxSize == Q->rear) //表明在队尾，不能继续取下一个元素了。
		return idx;
	else
	    return (idx +1)%Q->MaxSize;
}

int getPreviousIndex(Queue* Q, int idx)
{
	if ((idx )%Q->MaxSize == Q->front) //表明在队头了，不能继续取前面的元素。
		return idx;
	else
	    return (idx - 1)%Q->MaxSize;
}

/**
 * @brief 获取队列中从该索引到队尾元素的数量
 * 
 * @param Q 
 * @param idx 队列索引
 * @return int 元素的数量
 */

int QueueLengthToRear(Queue*Q,int idx)
{
	int len = 0;
	len=(Q->rear-idx+Q->MaxSize)%Q->MaxSize;
	return len;
}

/**
 * \brief 获取队列尾元素的索引.
 * 
 * \param Q 队列指针。
 * \return 返回队列尾元素的索引，若队列为空则返回-1。
 */
int getRearElemIndex(Queue* Q) {//获取队列尾元素的索引
	if (Q->front == Q->rear)
		return -1; //队空
	else
		return (Q->rear - 1 + Q->MaxSize) % Q->MaxSize; //返回尾元素的索引
}

/**
 * \brief 返回队列队头起第N个元素的地址，N=0，1,2，....
 * 
 * \param Q 队列指针。
 * \param N 队列元素的下标。
 * \return  返回的节点元素地址。
 */
void *GetQueueElem( Queue* Q, int N) {//返回Q的第N个元素地址，不修改队列。
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char *)Q->base + ((Q->front + N)% Q->MaxSize) * Q->ElemSize;
	if (Q->front == Q->rear)
		goto ERR; //队空	
	ReleaseMutex(Q->hMutex);
	return addr;
ERR:
	ReleaseMutex(Q->hMutex);
	return NULL;
}

/**
 * \brief 返回队列队头起第N个元素的地址，N=0，1,2，....
 *
 * \param Q 队列指针。
 * \param N 队列元素的下标。
 * \return  返回的节点元素地址。
 */
void* GetQueueElemISR(Queue* Q, int N) {//返回Q的第N个元素地址，不修改队列。
	char* addr = (char*)Q->base + ((Q->front + N) % Q->MaxSize) * Q->ElemSize;
	if (Q->front == Q->rear)
		goto ERR; //队空	
	return addr;
ERR:
	return NULL;
}

/**
 * \brief 设置下标为N的元素，N=1,2，....
 *
 * \param Q 队列指针。
 * \param N 队列元素的下标。
 * \param e 设置的元素。
 * \return
 */
int SetQueueElem( Queue* Q, int N, void* e) {//设置下标为N的元素，不修改队头指针
	WaitForSingleObject(Q->hMutex, INFINITE);
	char* addr = (char*)Q->base + ((Q->front + N) % Q->MaxSize) * Q->ElemSize;
    if (Q->front == Q->rear)
		goto ERR; //队空	
	memcpy(addr, e, Q->ElemSize);
	ReleaseMutex(Q->hMutex);
	return 0;
ERR:
	ReleaseMutex(Q->hMutex);
	return 1;
}
