/* 
** Copyright (C) 2019 Arthur BOUTON
** 
** This program is free software: you can redistribute it and/or modify  
** it under the terms of the GNU General Public License as published by  
** the Free Software Foundation, version 3.
**
** This program is distributed in the hope that it will be useful, but 
** WITHOUT ANY WARRANTY; without even the implied warranty of 
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
** General Public License for more details.
**
** You should have received a copy of the GNU General Public License 
** along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OSGKEYBOARD_H
#define OSGKEYBOARD_H

#include <osgGA/GUIEventHandler>
#include <osgViewer/View>


namespace renderer
{


class KeyboardEventHandler : public osgGA::GUIEventHandler
{
	public:

	typedef void (*functionType)( void* );

	enum keyStatusType { KEY_UP, KEY_DOWN };

	struct functionStatusType
	{
		functionStatusType() { keyState = KEY_UP; keyFunction = NULL; functionArg = NULL; }
		keyStatusType keyState;
		functionType keyFunction;
		void* functionArg;
	};

	KeyboardEventHandler( osgViewer::View* view = NULL );

	osgViewer::View* get_view();
	void set_view( osgViewer::View* view );

	bool isUsed( int whatKey );
	void addFunction( int whatKey, functionType newFunction, void* arg = NULL );  
	void addFunction( int whatKey, keyStatusType keyPressStatus, functionType newFunction, void* arg = NULL );  

	virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& );  
	virtual void accept( osgGA::GUIEventHandlerVisitor& v ) { v.visit( *this ); };  

	inline bool paused() { return _paused; }
	inline void set_pause() { _paused = true; _space_switch = true; }
	inline bool do_single_step()
	{
		if ( _do_single_step )
		{
			_do_single_step = false;
			return true;
		}
		else
			return false;
	}
	inline float get_warp_factor() { return _warp_factor; }

	protected:  

	typedef std::map<int, functionStatusType> keyFunctionMap;

	keyFunctionMap _keyFuncMap;
	keyFunctionMap _keyUPFuncMap;  

	bool _paused;
	bool _do_single_step;

	bool _space_switch;
	bool _n_state;

	float _warp_factor;

	osgViewer::View* _view;
};


}

#endif
