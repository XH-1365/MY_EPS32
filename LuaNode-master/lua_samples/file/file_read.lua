-- Read an existing file

res = file.open("test.lua", "r");

while true do
  if(res == false) then 
    print("Open file failed");
    break;
  end
  
  -- read 64 character from the file
  content = file.read(64);
  print(content);
  file.close();

  break;
end
