#ifndef  __STACK_H__
#define  __STACK_H__

typedef int DataType;
typedef struct node{
    DataType data;
    struct node * next;
}Stack;

Stack* CreateStack(void);
int IsEmpty(Stack* stack);
int PushStack(Stack* stack, DataType data);
int PopStack(Stack* stack);
void StackEmpty(Stack* stack);
DataType GetTopElement(Stack* stack);
void DestoryStack(Stack* stack);

#endif

