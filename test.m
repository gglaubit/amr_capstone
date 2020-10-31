
bags = dir('*.bag'); % this calls/lists all bag files in directory
tester = zeros(length(bags), 2);
topic1 = {'/gazebo/model_states'};

for i = 1:length(bags)
    name = convertStringsToChars(bags(i).name);
    r = strfind(name,'n');
    index = r
    %index = r(length(r)-1);
    n = extractBetween(name,index+1,length(name)-4);
    run = str2num(n{1});
    
    bag = rosbag(bags(i).name);
    bSel1 = select(bag,'Topic', topic1);
    msgStructs1 = readMessages(bSel1,'DataFormat','struct');
    s = size(msgStructs1{1}.Name);
    n = s(1);
    row = [run, n];
    tester(i,:) = row;
end

csvwrite('tester.csv', tester)