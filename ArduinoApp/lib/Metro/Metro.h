#ifndef Metro_h
#define Metro_h

class Metro
{

public:
  Metro();
  void setInterval(unsigned long interval);
  bool check();
	
private:
  unsigned long  previous, interval;

};

#endif