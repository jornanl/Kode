function sortedList = insertionSort(A);
%Insertion sort
for i=2:1:size(A,1)
    value = A(i,:);
    j = i-1;
    while j>=0 && A(j,1)>value(:,1)
        A(j+1,:) = A(j,:);
        j = j-1;
    end
    A(j+1,:) = value;
end
sortedList = A;
end