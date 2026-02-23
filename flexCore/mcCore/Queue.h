/*****************************************************************//**
 * \file   Queue.h
 * \brief  
 * 
 * \author Libing
 * \date   March 2022
 *********************************************************************/

#ifndef QUEUE_H_
#define QUEUE_H_
#ifdef __cplusplus
extern "C" {
#endif
 #include "platformCfg.h"

	///////////////////////////////////////////////////
	/**
	 * \brief 通用队列结构体定义,适用于各种数据结构.
	 */
	typedef struct {
		HANDLE hMutex; ///<互斥量.
        int lock;
		void* base;    ///<指向数据段的基地址(起始地址).
		int ElemSize;  ///<单个元素所占的存储空间，单位字节。
		int MaxSize;   ///<队列存储空间大小，注意，队列最大元素个数比存储空间小1，MaxSize-1.
		int FreeSize;  ///<队列剩余空间大小。
		int front;	   ///<队列头索引。
		int rear;	   ///<队列尾索引。
		// int current;     ///<当前指针，指向队列的当前元素。
		// int index1; 	///<当前指针的索引1, 供外部某个处理函数专用;
		// int index2; 	///<当前指针的索引2, 供外部某个处理函数专用;
		// int index3; 	///<当前指针的索引3, 供外部某个处理函数专用;
	}Queue;

	//循环队列的初始化。
	int InitQueue(Queue* Q, void* e, int MaxSize, int MemSize);
	int InitQueueISR(Queue* Q, void* e, int MaxSize, int ElemSize); //构造一个空队列Q
	//清空队列
	int ClearQueueISR(Queue* Q);
	//求循环队列的长度。
	int QueueLength(Queue* Q);
	int QueueLengthISR(Queue* Q);
	//求循环队列剩余空间.
	int QueueFreeSize(Queue* Q);
	//循环队列的入队。
	int EnQueueISR(Queue* Q, void* e);
	int EnQueue(Queue* Q, void* e);
	//循环队列的出队,删除Q的队头元素，用e返回其值。
	int DeQueue(Queue* Q, void* e);
	//中断内使用)循环队列的出队,删除Q的队头元素，用e返回其值.
	int DeQueueISR(Queue* Q, void* e);
	//返回Q的队头元素，不修改队头指针。
	int GetHeadElem(Queue* Q, void* e);
	int GetHeadElemISR(Queue* Q, void* e);

	void *GetHeadPtr(Queue* Q);
	//返回Q的第N个元素，不删除。
	void* GetQueueElem(Queue* Q, int N);
	/**
	 * \brief 返回队列队头起第N个元素的地址，N=0，1,2，....
	 *
	 * \param Q 队列指针。
	 * \param N 队列元素的下标。
	 * \return  返回的节点元素地址。
	 */
	void* GetQueueElemISR(Queue* Q, int N);
	//设置下标为N的元素，N=0,1,2，....
	int SetQueueElem(Queue* Q, int N, void* e);

	void* getPreviousElem(Queue* Q,int *idx);
	void* getCurrentElem(Queue* Q,int idx);
	// void* getNextElem(Queue* Q, int *idx);
	int isQueueRear(Queue* Q, int idx);
	int isQueueHead(Queue* Q, int idx);
	int getNextIndex(Queue* Q, int idx);
	int getPreviousIndex(Queue* Q, int idx);
	void* getElemFromRear( Queue* Q,int idx);
	void* getElemFromHead( Queue* Q,int idx);

	/**
	 * @brief 获取队列中从该索引到队尾元素的数量
	 * 
	 * @param Q 
	 * @param idx 队列索引
	 * @return int 元素的数量
	 */

	int QueueLengthToRear(Queue*Q,int idx);
	
	/**
	 * \brief 获取队列尾元素的索引.
	 * 
	 * \param Q 队列指针。
	 * \return 返回队列尾元素的索引，若队列为空则返回-1。
	 */
	int getRearElemIndex(Queue* Q) ;

#ifdef __cplusplus
}
#endif

#endif // !QUEUE_H_