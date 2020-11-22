#ifndef xmlio_h
#define xmlio_h
/*! \mainpage XMLio - Reference
 *
 * \section intro_sec Introduction
 *
 * The XMLio library is a simply and lightweight XML Parser and encoder.
 *
 * \section install_sec Installation
 *
 * Just type make.
 *
 * \section usage_sec Example program
 *
 * \include xmlview.cpp
 */



#include <string>
#include <vector>
#include <map>
#include <fstream>
#include "textout.h"

#ifdef WIN32
#ifndef MINGW
#define DllExport __declspec( dllexport )
#pragma warning (disable : 4251)
#else
#define DllExport
#endif
#else 
#define DllExport
#endif

class XMLParser;
class XMLError;

/** @defgroup xml XML Core
 *  Use XMLParser to open an XML file, XMLNode to navigate through the parsed tree.
 */


/** @ingroup xml
 *  @brief A node for XML data.
 *
 *  A single XML node, which can contain attributes, text and more XML nodes.
 */
class DllExport XMLNode {
public:
	/** Constructor */
	XMLNode(int t=0);

	/** Constructor */
	XMLNode(const std::string &str);

	/** Constructor */
	XMLNode(const std::string &str, const std::string &t);

	/** Destructor */
	virtual ~XMLNode();

	/** Start parsing. */
	virtual bool Parse(std::istream &is, XMLParser *parser);
	/** Write XML tree. */
	friend DllExport std::ostream &operator <<(std::ostream &os, const XMLNode &node);

	void Debug(std::ostream &os, TextOutputType_e e=ASCII);

	/** @name Get child nodes
	 * @{
	 */
	/** Get first child matching node name.
	 * @param name Node name
	 */
	XMLNode *GetFirstChild(const std::string &name) ;
	/** Get next child matching node name.
	 * @param name Node name
	 */
	XMLNode *GetNextChild(const std::string &name) ;
	/** Get first child.
	 */
	XMLNode *GetFirstChild() ;
	/** Get next child.
	 */
	XMLNode *GetNextChild() ;
	/** Get child node with nodename name and "type"-attribute type. */
	XMLNode *GetChildWithType(const std::string &name, const std::string &type) const;
	/** Get child node with nodename name and "type"-attribute id. */
	XMLNode *GetChildWithId(const std::string &name, const std::string &id) const;
	/** Get child node with nodename name and "type"-attribute type. */
	XMLNode *GetTypedChild(int type) const;
	// @}


	/** @name Get node information
	 * @{
	 */
	/** Get name of the node. */
	std::string GetName() const {
		return nodename;
	}
	/** Get optional text between opening and closing tag. */
	std::string GetText() const {
		return text;
	}
	/** Get value of attribute. */
	std::string GetAttribute(const std::string &att) const;
	// @}


	/** @name Set node information
	 * @{
	 */
	/** Set name of the node. */
	void SetName(const std::string &str) {
		nodename=str;
	}
	/** Set optional text between opening and closing tag. */
	void SetText(const std::string &str) {
		text=str;
	}
	/** Set value of attribute. */
	void SetAttribute(const std::string &att, const std::string &value) {
		attribute[att]=value;
	}
	// @}

	/** @name Changing XML tree structure
	 * @{
	 */
	/** Add child node. */
	XMLNode *AddChild(XMLNode *n) {
		child.push_back(n);
		return n;
	}
	/** Add comment. */
	XMLNode *AddComment(const std::string &t) {
		XMLNode *n = new XMLNode("",t);
		n->type=2;
		child.push_back(n);
		return n;
	}

	/** Add child node. */
	XMLNode *AddChild(const std::string &str) {
		XMLNode *n = new XMLNode(str);

		child.push_back(n);
		return n;
	}
	/** Add child node. */
	XMLNode *AddChild(const std::string &str, const std::string &t) {
		XMLNode *n = new XMLNode(str,t);

		child.push_back(n);
		return n;
	}
	/** Remove first child node matching name t. */
	bool RemoveChild(const std::string &t);

	/** Remove all childs. */
	void RemoveAllChilds();

	/** Insert Child Node. */
	XMLNode* AddChildIfNotExists(const std::string &name);
	/** Insert Child Node. */
	XMLNode* AddChildIfNotExists(const std::string &name, const std::string &text);
	/** Extract value of node. */
	void GetChildValueIfExists(const std::string &name, const std::string Map[], int &value);
	void GetChildValueIfExists(const std::string &name, const std::vector<char*> &Map, int &value);
	void GetChildValueIfExists(const std::string &name, const char Map[3][10], int &value);



	/** Create full copy.
	  */
	XMLNode *Clone();

	// @}
	
	/** false, true */
    static char BOOLEAN[3][10];
	/** no, yes */
	static char LOGICAL[3][10];
	/** off, on */
	static char SWITCH[3][10];

	/** Create Top Level Node for a new XML tree. 
	 */
	static XMLNode* CreateRoot(const std::string &name);

protected:
	std::string nodename;
	int type;
	std::string text;
	std::vector<XMLNode*> child;
	std::map<std::string,std::string> attribute;

	std::vector<std::string> attrsplit(const std::string &s) const;
	bool isSplitCharacter(char c) const;
	bool attrparse(XMLParser *parser, std::vector<std::string> &ss);
	bool doctypeparse(XMLParser *parser, const std::string &a);

private:
	std::vector<XMLNode*>::const_iterator it_tmp;
};



/** @ingroup xml
  * @brief A parser for XML files.
  *
  * The parser opens XML files and reads them into XMLNodes. */
class DllExport XMLParser {
	friend class XMLNode; // for getline
public:
	/** Constructor */
	XMLParser();

	/** Destructor */
	~XMLParser();

	/** @name Set node information
	 * @{
	 */
	/** Start parsing process.
	 * @param filename Full path and filename of XML file.
	 * @return Root node of parsed file or 0 if XML file doesn't exist.
	 * */
	XMLNode *Parse(const std::string &filename);
	XMLNode *ParseString(const std::string &filename);
	XMLNode *ParseStream(std::istream &is);

	/** Return declaration node. */
	XMLNode *GetXMLDeclaration() const;

	/** Return doctype node. */
	XMLNode *GetDoctype() const;

	/** Return document node. */
	XMLNode *GetDocument() const;

	/** Test, if end of parsed file is reached. */
	bool EndOfFile(std::istream &is) const;
	// @}

	/** @name Error Handling
	 * @{
	 */
	/** Create error with code 'c'. */
	void AddError(int c, const std::string &s=std::string(""));

	/** ostream all occured errors. */
	void GetError(std::ostream &os) const;
	// @}

protected:
	XMLNode *root;

	//	std::istream* is;
	std::vector<XMLError *> xmlerrors;

	/** Get line of currently parsed file until c is reached. */
	std::string getline(std::istream &is, char c);
	int line;

};


std::ostream DllExport &operator <<(std::ostream &os, const std::map<std::string,std::string> &m);


#endif
