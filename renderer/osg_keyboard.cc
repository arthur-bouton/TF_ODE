#include "osg_keyboard.hh"

#include <iostream>


namespace renderer
{


KeyboardEventHandler::KeyboardEventHandler( osgViewer::View* view ) : _paused( false ),
																	  _do_single_step( false ),
											   				   		  _space_switch( false ),
											   				   		  _n_state( KEY_UP ),
															   		  _view( view ),
																	  _warp_factor( 1 ) {}

osgViewer::View* KeyboardEventHandler::get_view() { return _view; }

void KeyboardEventHandler::set_view( osgViewer::View* view ) { _view = view; }


bool KeyboardEventHandler::isUsed( int whatKey )
{
	if ( _keyFuncMap.find( whatKey ) != _keyFuncMap.end() )
		return true;
	return false;
}


void KeyboardEventHandler::addFunction( int whatKey, functionType newFunction, void* arg )
{
	_keyFuncMap[whatKey].keyFunction = newFunction;
	_keyFuncMap[whatKey].functionArg = arg;
}


void KeyboardEventHandler::addFunction( int whatKey, keyStatusType keyPressStatus, functionType newFunction, void* arg )
{
	if ( keyPressStatus == KEY_DOWN )
		return addFunction( whatKey, newFunction, arg );
	else
	{
		_keyUPFuncMap[whatKey].keyFunction = newFunction;
		_keyFuncMap[whatKey].functionArg = arg;
	}
}


bool KeyboardEventHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
	switch( ea.getEventType() )
	{
		case ( osgGA::GUIEventAdapter::KEYDOWN ):
		{
			keyFunctionMap::iterator itr = _keyFuncMap.find( ea.getKey() );
			if ( itr != _keyFuncMap.end() && (*itr).second.keyState == KEY_UP )
			{
				(*itr).second.keyState = KEY_DOWN;
				(*itr).second.keyFunction( (*itr).second.functionArg );
			}
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Space )
				_paused = true;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_C && _space_switch )
				_paused = false;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_N && _paused && _n_state == KEY_UP )
			{
				_do_single_step = true;
				_n_state = KEY_DOWN;
			}
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_I )
				_warp_factor /= 2;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_O )
				_warp_factor  = 1;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_P )
				_warp_factor *= 2;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_H )
			{
				if ( _view != NULL )
					_view->home();
			}
			else
				return false;

			return true;
		}
		case ( osgGA::GUIEventAdapter::KEYUP ):
		{
			keyFunctionMap::iterator itr = _keyFuncMap.find( ea.getKey() );
			if ( itr != _keyFuncMap.end() )
				(*itr).second.keyState = KEY_UP;

			itr = _keyUPFuncMap.find( ea.getKey() );
			if ( itr != _keyUPFuncMap.end() )
				(*itr).second.keyFunction( (*itr).second.functionArg );

			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Space )
			{
				if ( _space_switch )
					_paused = false;
				_space_switch = !_space_switch;
			}
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_C && _space_switch )
				_paused = true;
			else if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_N && _paused )
				_n_state = KEY_UP;
			else
				return false; 

			return true;
		}
		default:
			return false;
	}
}


}
