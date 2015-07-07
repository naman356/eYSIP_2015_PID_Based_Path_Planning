


// global variable to count the number of overflows
volatile uint8_t tot_overflow;


void timer4_init(void)
{
	tot_overflow=0;
	TCCR4B = 0x00; //stop
	TCNT4H = 0xC6; //Counter higher 8 bit value
	TCNT4L = 0x67; //Counter lower 8 bit value
	OCR4AH = 0x00; //Output compare Register (OCR)- Not used
	OCR4AL = 0x00; //Output compare Register (OCR)- Not used
	OCR4BH = 0x00; //Output compare Register (OCR)- Not used
	OCR4BL = 0x00; //Output compare Register (OCR)- Not used
	OCR4CH = 0x00; //Output compare Register (OCR)- Not used
	OCR4CL = 0x00; //Output compare Register (OCR)- Not used
	ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
	ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
	TCCR4A = 0x00;
	TCCR4C = 0x00;
	TCCR4B = 0x01; //start Timer
}

// TIMER4 overflow interrupt service routine
// called whenever TCNT4 overflows
ISR(TIMER4_OVF_vect)
{
	// keep a track of number of overflows
	tot_overflow++;
	//TIMER4 has overflowed
	TCNT4H = 0xC6; //reload counter high value
	TCNT4L = 0x67; //reload counter low value
}

void start_timer4(void)
{
	cli(); //Clears the global interrupts
	timer4_init();
	TIMSK4 = 0x01; //timer4 overflow interrupt enable
	sei();   //Enables the global interrupts

}

int millis(void)
{
	int time =0;
	time=tot_overflow + (TCNT4-50791)/14745;
	start_timer4();
	return time;

}



