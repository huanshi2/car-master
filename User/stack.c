#include <stdlib.h>
#include <stdio.h>
#include <stack.h>


Stack* CreateStack()
{
    Stack *stack = (Stack*)malloc(sizeof(Stack));
    if(NULL != stack)
    {
       stack->next = NULL;
       return stack;
    }
    printf("out of place.\n");
    return NULL;
}

int IsEmpty(Stack* stack)
{
    return (stack->next == 0);
}
  

//��ջ
int PushStack(Stack* stack, DataType data)
{
    Stack* newst = (Stack*)malloc(sizeof(Stack));
    if(NULL != newst)
    {
        newst->data = data;
        newst->next = stack->next;  //s->next = NULL;
        stack->next = newst;
        return 1;
    }
    printf("out of place PushStack.\n");
    return 0;
}
 
/*
    ��ջ
 */
 
int PopStack(Stack* stack)
{
    Stack* tmpst;
    if(!IsEmpty(stack))
    {
        tmpst = stack->next;
        stack->next = tmpst->next;
        free(tmpst);
        return 1;
    }
    return 0;
}
 
//��ȡջ��Ԫ��
DataType GetTopElement(Stack* stack)
{
    if(!IsEmpty(stack))
    {
        return stack->next->data;
    }
    printf("stack is empty GetTopElement.\n");
    return -1;
}


//���ջ
void StackEmpty(Stack* stack)
{
    while(!IsEmpty(stack))
    {
        PopStack(stack);
    }
    printf("now stack is empty. \n");
}

//����ջ
void DestoryStack(Stack* stack)
{
    free(stack);
    printf("now stack is destoryed. \n");
}




