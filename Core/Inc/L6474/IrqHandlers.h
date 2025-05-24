#ifndef IRQ_HANDLERS_H
#define IRQ_HANDLERS_H



struct Tim3IrqHandler
{
	void (*func)(void*);
	void *userptr;
};


struct IrqHandler
{
	void (*func)();
	int enabled;
};

struct IrqHandlers
{
	struct Tim3IrqHandler tim3;
	struct IrqHandler irq;
};

#endif
