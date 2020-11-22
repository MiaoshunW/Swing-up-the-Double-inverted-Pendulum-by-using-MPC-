//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef toolstatus_h
#define toolstatus_h

#include <vector>
#include <string>

/** @ingroup toolbase
 *  @brief Display of status information.
 *
 * Self-destructing status information at bottom of OpenGL window.
 */
class ToolStatus {

public:
	/** Constructor. */
	ToolStatus() {}
	
	/** Destructor. */
	virtual ~ToolStatus() {}
	
	/**
	 * Draw status information.
	 * @param height Height of OpenGL window.
	 */
	void Draw(int height) const;
	
	/**
	 * Timer function, clean up old status information.
	 * @param time Current time.
	 */
	void Timer(int time);
	
	/**
	 * Add line to status information.
	 * @param s Text to add.
	 * @param time Current time.
	 */
	void Add(const std::string &s, int time);

void Clear();
private:
	struct Statusline {
		Statusline(const std::string &t, int tt) : text(t), time(tt) {}
		std::string text;
		int time;
	

	};
	std::vector<Statusline> statusline;
};

#endif
