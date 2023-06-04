#include <iostream>
#include <fstream>
#include <stdio.h>
using namespace std;

int main() {
    ofstream file;
    string filename;
    int bytes;
    int val;

    system("clear");
    
    cout << "File name: ";
    cin >> filename;

    file.open(filename, ios::out | ios::binary | ios::trunc);

    cout << "How many bytes: ";
    cin >> bytes;
    cout << "Of what value: ";
    cin >> val;
    cout << val << endl;

    char vals[bytes];
    for (int i = 0; i < bytes; i++) {
        vals[i] = val;
    }
    file.write(vals, bytes);

    cout << "done" << endl;

    file.close();

    
}
