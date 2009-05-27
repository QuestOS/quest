#include <stdio.h>

#define SIZE 10000

static char a[ SIZE ];

static void remove_multiples( int k ) {

    int i;

    for( i = k * 2; i < SIZE; i += k )
	a[ i ] = 0;
}

int main() {

    int i, iter;

    for( iter = 0; iter < 10000; iter++ ) {
	for( i = 2; i < SIZE; i++ )
	    a[ i ] = 1;
    
	for( i = 2; i * i < SIZE; i++ )
	    if( a[ i ] )
		remove_multiples( i );
    }

    printf ("finished!\n");

    return 0;
}
