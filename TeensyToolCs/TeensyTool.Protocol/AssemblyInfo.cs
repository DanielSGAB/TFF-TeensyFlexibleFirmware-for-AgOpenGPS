using System.Runtime.CompilerServices;

// InternalsVisibleTo: talar om för kompilatorn "gör allt som är
// märkt 'internal' i det här projektet synligt för det angivna,
// namngivna projektet också — men fortfarande INTE för någon annan".
//
// Varför vi vill det: ProcessIncomingSentence() i TeensyConnection.cs
// är avsiktligt "internal", inte "public", eftersom den bara är en
// implementationsdetalj — ingen som faktiskt ANVÄNDER vårt bibliotek
// (ett framtida Forms-fönster) ska behöva bry sig om den, bara om den
// riktiga, publika DataReceived-händelsen. Men vi VILL kunna testa
// den isolerat, utan en riktig socket, i vårt eget testprojekt — den
// här raden löser exakt den motsättningen.
[assembly: InternalsVisibleTo("TeensyTool.Protocol.Tests")]
