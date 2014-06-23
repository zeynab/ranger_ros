/*
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2012:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ASEBACONNECTOR_H
#define ASEBACONNECTOR_H

#include <vector>
#include <string>
#include <dashel/dashel.h>
#include "common/msg/descriptions-manager.h"

struct AsebaConnector: public Dashel::Hub, public Aseba::DescriptionsManager
{
public:
	typedef std::vector<std::string> strings;
	typedef std::map<std::string, Aseba::VariablesMap> NodeNameToVariablesMap;
	
protected:
	Dashel::Stream* targetStream;
	
	// result of last compilation and load used to interprete messages
	Aseba::CommonDefinitions commonDefinitions;
	NodeNameToVariablesMap allVariables;
	
public:
	// interface with main()
	AsebaConnector(const char* target);
    bool isValid() const;

    int l_encoder, r_encoder;

protected:
	// reimplemented from parent classes
	virtual void incomingData(Dashel::Stream *stream);
	
	void emit(const strings& args);
};

#endif
