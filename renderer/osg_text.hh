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

#ifndef OSGTEXT_HH
#define OSGTEXT_HH

#include <osgText/Text>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <memory>


namespace renderer 
{

  
class OsgText
{
	public:

	typedef std::shared_ptr<OsgText> ptr_t;

	OsgText( osg::ref_ptr<osg::Group> root, int window_width = 1024, int window_height = 768, int alignment = 1 );

	void set_alignment( int alignment );

	void set_pos( int x, int y );

	inline int get_x() const { return _x; }
	inline int get_y() const { return _y; }

	inline void set_text( const std::string& text )
	{	
		_text->setText( text, osgText::String::ENCODING_UTF8 );
	}

	void set_size( float size )
	{	
		_text->setCharacterSize( size );
	}

	void set_font( const std::string& font_path )
	{	
		_text->setFont( font_path );
	}

	void set_color( float r, float g, float b, float alpha = 1 )
	{	
		_text->setColor( osg::Vec4( r, g, b, alpha ) );
	}

	void add_background( float margin = 10, float alpha = 0.3, float r = 10, float g = 10, float b = 10 );

	void set_callback( std::function<bool(OsgText*)> callback )
	{	
		_callback = callback;
	}

	inline bool update()
	{	
		if ( _callback != nullptr )
			return _callback( this );
		return false;
	}

	~OsgText();

	protected:    

	osg::ref_ptr<osgText::Text> _text;
	int _width, _height, _x, _y;

	std::function<bool(OsgText*)> _callback;

	osg::ref_ptr<osg::Group> _root;
	osg::ref_ptr<osg::Geode> TextGeode;
	osg::ref_ptr<osg::Projection> TextProjectionMatrix;
	osg::ref_ptr<osg::MatrixTransform> TextModelViewMatrix;
};


}


#endif
