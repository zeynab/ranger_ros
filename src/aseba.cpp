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

#include <iostream>
#include <sstream>
#include <iterator>
#include <cstdlib>
//#include <libxml/parser.h>
//#include <libxml/tree.h>

#include "aseba.h"
#include <common/utils/utils.h> // Aseba's UTF8ToWString

using namespace std;
using namespace Dashel;
using namespace Aseba;

// code of the shell

AsebaConnector::AsebaConnector(const char* target):
	// connect to the Aseba target
	targetStream(connect(target))
{
	// request a description of the target
	GetDescription getDescription;
	getDescription.serialize(targetStream);
	targetStream->flush();
	

}

bool AsebaConnector::isValid() const
{
	return targetStream;
}

void AsebaConnector::incomingData(Dashel::Stream *stream)
{
	// receive message
	Message *message = 0;
	try
	{
		// deserialize message using Aseba::Message::receive() static function
		message = Message::receive(stream);
	}
	catch (DashelException e)
	{
		// if this stream has a problem, ignore it for now,
		// and let Hub call connectionClosed later.
		wcout << "error while receiving message: " << e.what() << endl;
		return;
	}

	// pass message to description manager, which builds
	// the node descriptions in background
	DescriptionsManager::processMessage(message);
	
	// if user event, print
	const UserMessage *userMessage(dynamic_cast<UserMessage *>(message));
	if (userMessage)
	{
		wcout << '\r';
		if (userMessage->type < commonDefinitions.events.size())
			wcout << commonDefinitions.events[userMessage->type].name;
		else
			wcout << "unknown event " << userMessage->type;
		wcout << ": ";
		for (size_t i = 0; i < userMessage->data.size(); ++i)
			wcout << userMessage->data[i] << " ";
		wcout << endl;

        if (userMessage->type == 1) {
            l_encoder = -userMessage->data[15];
            r_encoder = userMessage->data[13];
        }
    }
	
	delete message;
}

void AsebaConnector::emit(const strings& args)
{
	// check that there are enough arguments
	if (args.size() < 2)
	{
		wcout << "missing argument, usage: emit EVENT_NAME EVENT_DATA*" << endl;
		return;
	}
	size_t pos;
	if (!commonDefinitions.events.contains(UTF8ToWString(args[1]), &pos))
	{
		wcout << "event " << UTF8ToWString(args[1]) << " is unknown" << endl;
		return;
	}
	
	// build event and emit
	UserMessage::DataVector data;
	for (size_t i=2; i<args.size(); ++i)
		data.push_back(atoi(args[i].c_str()));
	UserMessage userMessage(pos, data);
	userMessage.serialize(targetStream);
	targetStream->flush();
}

int toto_main(int argc, char *argv[])
{
	// check command line arguments
	if (argc != 2)
	{
		wcout << "Usage: " << argv[0] << " TARGET" << endl;
		return 1;
	}
	
	// create the shell
	AsebaConnector shell(argv[1]);
	// check whether connection was successful
	if (!shell.isValid())
	{
		wcout << "Connection failure" << endl;
		return 2;
	}
	
	// run the Dashel Hub
	shell.Hub::run();
	
	return 0;
}
