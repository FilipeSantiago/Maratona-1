Maratona
========
## Container
```cpp
var.clear(); // Remove todos elementos

bool vazio = var.empty(); // Vazio?
unsigned in tamanho = var.size(); // Tamanho

for (; it != var.end(); it++) //Loop
```

## vector
```cpp
#include <vector>

// Declaração
vector<TIPO> var; 
vector<TIPO> var(QTD, VALOR);

var.push_back(VALOR);  // Adiciona elemento
var.pop_back(); // Remove elemento

vector<TIPO>::iterator it = var.begin(); // Iterador normal
vector<TIPO>::reverse_iterator rit = var.rbegin(); // Iterador reverso

var.insert(it, VALOR); // Insere na posição it
var.insert(it, QTD, VALOR); // Insere QTD VALORES na posição it
var.insert(it, ARRAY + 0, ARRAY + Tamanho); // Insere todos os valores de um array.

TIPO primeiro = var.front(); // Primeiro elemento
TIPO ultimo = var.back(); // Último elemento
```

## deque << vector
```cpp
#include <deque>

// Declaração
deque<TIPO> var;
var.push_front(VALOR); // Adiciona no inicio
var.pop_front(); // Remove do inicio
```

## map
```cpp
#include <map>
map<TIPO_KEY, TIPO_VALUE> var;

var.insert(make_pair(KEY, VALUE)); // Insere
var.erase(KEY); // Remove

TIPO_VALUE valor = var[KEY]; // Acessa mapa
map<int, int>::iterator key_value = var.find(KEY); // Busca elemento
bool existe = key_value != var.end(); // Existe?
```

## multimap
```cpp
#include <map>
multimap<TIPO_KEY, TIPO_VALUE> var;

var.insert(make_pair(KEY, VALUE)); // Insere
var.erase(KEY); // Remove todos os valores associados à KEY
multimap<TIPO_KEY, TIPO_VALUE>::iterator it = ...;
var.erase(it); // Remove uma entrada

multimap<int, int>::iterator key_value = var.find(KEY); // Busca alguma entrada
bool existe = key_value != var.end(); // Existe?

//Percorrimento das entradas de uma chave
for(multimap<TIPO_KEY, TIPO_VAL>::iterator it = var.lower_bound(KEY); it != var.upper_bound(KEY); it++){
	TIPO_KEY key = it->first;
	TIPO_VALUE value = it->second;
}
```

<div></div>

## set
```cpp
#include <set>
set<TIPO> var;

var.insert(ITEM); // Insere
var.erase(KEY); // Remove
bool existe = var.find(ITEM) != var.end(); // Contém?
```

<br><br><br><br>

## queue
```cpp
#include <queue>
queue<TIPO> var;

var.push(ITEM); // Insere
TIPO primeiro = var.front(); // Pega o primeiro sem remover
var.pop(); // Remove primeiro
```

## priority queue
```cpp
#include <queue>
priority_queue<TIPO> var; // Cria com comparador padrão Less<T>

class MyComparator{
	bool operator ()(const TIPO & v1, const TIPO & v2){...}
}

priority_queue<TIPO, std::vector<TIPO>, MyComparator> var; // Cria com comparador personalizado

var.push(ITEM); // Insere
TIPO primeiro = var.top(); // Pega o primeiro sem remover
var.pop(); // Remove primeiro
```

## stream
```cpp
istream& operator>>(istream &input, TIPO &p) {
  input >> p.VALOR1 >> p.VALOR2;
  return input;
}
ostream& operator<<(ostream &out, TIPO &p) {
  out << p.VALOR1 << " " << p.VALOR2;
  return out;
}


string a, b, c;
cin >> a;
cin.ignore(); // Discartar espaços
getline(cin, b);
cin >> c;
```
<br><br><br><br>

## sstream
```cpp
#include <sstream>
stringstream ss;
stringstream ss("text");
ss << "2"; // Input
int num;
ss >> num; // Output
string result = ss.str() // Tudo o que está no stream
```

## iostream e iomanip
```cpp
#include <iostream>
// input
char a, b, c;
istringstream ss("  123")'
ss >> skipws >> a >> b >> c;
cout << a << b << c; // '123'
iss.seekg(0);
ss >> noskipws >> a >> b >> c;
cout << a << b << c; // '  1'
// Float
cout.precision(5); 
cout << f; // 3.14159
cout << fixed << n; // 2006.00000
cout << scientific << n; // 2.00600e+003
// Base
cout << uppercase << hex << n; //4D
cout << nouppercase << hex << n; // 4d
cout << showbase << hex << n; //0x4d
cout << dec << n;
cout << oct << n;
// Bool
cout << boolalpha << b; // true
cout << noboolalpha << b; // 1
// Justify
cout.width(6); cout << internal << n; // "-   77"
cout.width(6); cout << left << n;     // "-77   "
cout.width(6); cout << right << n;    // "   -77"

#include <iomanip>
// Float
cout << setprecision(5) << f; // 3.14159
// Base
cout << setbase(16) << n; // 4d
// Justify
cout << setw(6) << n; // "   -77"
// Fill
cout << setfill('x') << setw(6) << n; // "xxx-77"
```

## algorithm
```cpp
#include <algorithm>

// Básico
void imprime(int i) {cout << i << endl;}
bool menor(int i,int j) { return i<j; }
bool igual(int i,int j) { return i==j; }
bool par(int i){ return i%2==0; }
int dobrar(int i) { return 2*i; }
int aleatorio() { return rand()%100; }
vector<int> vet, vet2, ordenado;
// Considere <param> como opcional

// Percorre aplicando função
for_each(vet.begin(), vet.end(), imprime);

// Busca
vector<int>::iterator it;
it = find(vet.begin(), vet.end(), VALOR); // linear
it = find_if(vet.begin(), vet.end(), par); // linear com condição
it = binary_search(ordenado.begin(), ordenado.end(), VALOR, <menor>); // binária
it = lower_bound(ordenado.begin(), ordenado.end(), VALOR, <menor>); // ponteiro para o primeiro VALOR
it = upper_bound(ordenado.begin(), ordenado.end(), VALOR, <menor>); // ponteiro para o primeiro numero depois de VALOR
it = search(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // sublista
it = find_end(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // sublista no final
it = find_first_of(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // um dos resultados
it = adjacent_find(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // elementos repetidos
it = search_n(vet.begin(), vet.end(), QTD, VALOR); // QTD VALORES seguidos

pair<vector<int>::iterator,vector<int>::iterator> par;
par = mismatch(vet.begin(), vet.end(), vet2.begin()); // Primeira diferença
par = equal_range(ordenado.begin(), ordenado.end(), VALOR, <menor>); // Subconjunto de VALORES

// Contar
int c = count(vet.begin(), vet.end(), 10);
int c = count_if(vet.begin(), vet.end(), par);

// Igualgdade
bool igual = equal(vet.begin(), vet.end(), vet2.begin(), <igual>);

// Copiar
vet2.resize(vet.size());
copy(vet.begin(), vet.end(), vet2.begin()); 
copy_backward(vet.begin(), vet.end(), vet2.end()); // inverso

// Swap
swap(vet, vet2); // Swap variavel
swap_ranges(vet.begin() + 1, vet.end() - 1, vet2.begin()); // Swap parte do vetor
iter_swap(vet.begin(), vet2.begin() + 1); // Swap dentro de vetor.

// Transform
vet2.resize(vet.size());
transform(vet.begin(), vet.end(), vet2.begin(), dobrar); // mapeia dobrar para cada elemento de vet
transform(vet.begin(), vet.end(), vet2.begin(), result.begin(), soma); // mapeia soma para cada para <vet[i], vet2[i]>

// Substituir elementos
replace(vet.begin(), vet.end(), ORIGINAL, DESTINO);
replace_if(vet.begin(), vet.end(), par, DESTINO);
replace_copy(vet.begin(), vet.end(), vet2.begin(), ORIGINAL, DESTINO); // Resultado em vet2
replace_copy_if(vet.begin(), vet.end(), vet2.begin(), par, DESTINO); // Resultado em vet2

// Preencher
fill(vet.begin(), vet.begin() + 3, VALOR); // VALOR VALOR VALOR vet[4] vet[5] ...
fill_n(vet.begin(), 3, VALOR); // VALOR VALOR VALOR vet[4] vet[5] ...

// Gerar
struct c_unique { 
  int current;
  c_unique() {current=0;}
  int operator()() {return ++current;}
} UniqueNumber;

generate(vet.begin(), vet.end(), UniqueNumber);
generate_n(vet.begin(), 3, aleatorio);

// Remover elementos
it = remove(vet.begin(), vet.end(), 20); // "20 10 20 15" -> "10 15 ? ?". Retorna novo end
it = remove_if(ver.begin(), vet.end(), par);
remove_copy(vet.begin(), vet.end(), vet2.begin(), 20);
remove_copy_if(vet.begin(), vet.end(), vet2.begin(), par);

// Remover duplicatas consecutivas
it = unique(vet.begin(), vet.end(), <igual>); // "20 20 10 20" -> "20 10 20 ?". Retorna novo end
unique_copy(vet.begin(), vet.end(), vet2.begin(), <igual>); 

// Inverter
reverse(vet.begin(), vet.end());
reverse_copy(vet.begin(), vet.end(), vet2.end());

// Circular
rotate(vet.begin(), vet.begin() + 2, vet.end()); // "1 2 3 4" -> "3 4 1 2"
rotate_copy(vet.begin(), vet.begin() + 2, vet.end(), vet2.begin());

// Embaralhar
random_shuffle(vet.begin(), vet.end(), <aleatorio>);

// Separar elementos
it = partition(vet.begin(), vet.end(), par); // begin - it: pares; it - end: impares
it = stable_partition(vet.begin(), vet.end(), par); // mantem a ordem

// Ordenar
sort(vet.begin(), vet.end(), <menor>);
stable_sort(vet.begin(), vet.end(), <menor>); // mantem ordem de elementos semelhantes
partial_sort(vet.begin(), vet.begin() + 2, vet.end(), <menor>); // "4 3 2 1" -> "1 2 4 3"
partial_sort_copy(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <menor>);
nth_element(vet.begin(), vet.begin() + 5, vet.end(), <menor>); // vet[0..4] < vet[5] < vet[6..]

// Junção
merge(ordenado.begin(), ordenado.end(), ordenado2.begin(), ordenado2.end(), vet.begin());
inplace_merge(ordenado.begin(), ordenado.begin()+5, ordenado.end());
bool inclui = includes(ordenado.begin(), ordenado.end(), ordenado2.begin(), ordenado2.end());
it = set_union(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Merge sem repetição
it = set_intersection(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Interseção
it = set_difference(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Diferença
it = set_symmetric_difference(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // (A - B) U (B - A)

// Heap
make_heap(vet.begin(), vet.end());
pop_head(vet.begin(), vet.end()); vet.pop_back();
vet.push_back(99); push_heap(vet.begin(), vet.end());
cout << "max: " << vet.front();
sort_heap(vet.begin(), vet.end());

// Min/max
v = min(VALOR, VALOR2);
v = max(VALOR, VALOR2);
it = min_element(vet.begin(), vet.end(), <menor>);
it = max_element(vet.begin(), vet.end(), <menor>);

// Comparação lexicográfica
int cmp = lexicographical_compare(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <menor>); 

// Permutações, crescente
sort(vet.begin(), vet.end());
do {
	for_each(vet.begin(), vet.end(), imprime);
} while (next_permutation(vet.begin(), vet.end()));


// Permutações, decrescente
sort(vet.begin(), vet.end());
reverse(vet.begin(), vet.end());
do {
	for_each(vet.begin(), vet.end(), imprime);
} while (prev_permutation(vet.begin(), vet.end()));
```

## functional
```cpp
#include <functional>

// Aritmética / lógica
transform(vet.begin(), vet.end(), vet2.begin(), result.begin(), plus<TIPO>());
minus<TIPO>(); multiplies<TIPO>(); divides<TIPO>(); modulus<TIPO>(); negate<TIPO>();
logical_and<bool>(); logical_or<bool>(); logical_not<bool>();
// Comparação
it = mismatch(vet.begin(), vet.end(), vet2.begin(), equal_to<int>());
not_equal_to<TIPO>(); greater<TIPO>(); less<TIPO>(); greater_equal<TIPO>(); less_equal<TIPO>();
// Negação
count_if(vet.begin(), vet.end(), not1(par));
it = mismatch(vet.begin(), vet.end(), vet2.begin(), not2(equal_to<int>()));
// Bind
count_if(vet.begin(), vet.end(), bind1st(equal_to<int>(), 10));
count_if(vet.begin(), vet.end(), bind2nd(less<int>(), 10));
// Converte ponteiro em função
transform(vet.begin(), vet.end(), vet2.begin(), ptr_fun(atoi));
// Usar método de objeto
transform(vet.begin(), vet.end(), vet2.begin(), mem_fun(&string::length)); // se valores forem ponteiros
transform(vet.begin(), vet.end(), vet2.begin(), mem_fun_ref(&string::length)); // se valores forem valores

```

## utility
```cpp
#include <utility>
swap(a, b); 
pair<TIPO, TIPO> x = make_pair(a, b);
x.first; x.second;
```

## numeric
```cpp
#include <numeric>
// Acumular resultado - reduce
int v = accumulate(vet.begin(), vet.end(), INIT, <plus<int>()>); 
// Subtrair valores adjacentes
adjacent_difference(vet.begin(), vet.end(), vet2.begin(), <minus<int>()>);
// Acumular produtos entre adjacentes
int v = inner_product(vet.begin(), vet.end(), vet2.begin(), INIT, <plus<int>(), multiplies<int>()>)
// Soma parcial
partial_sum(vet.begin(), vet.end(), vet2.begin(), <plus<int>()>); // "1 2 3 4" -> "1 3 6 10"
```

## string
```cpp
#include <string>
string s;
string s("abc"); // construtor
string::iterator b = s.begin(), e = s.end(); // iteradores
int tamanho = s.size(); tamanho = s.length(); // tamanho
bool vazia = s.empty(); // vazia?
cout << c[1] << c.at(1); // acessar
s += "outra"; s.append("outra");
s.push_back('c');
char * cstr = s.c_str(); // converte para c string
// Busca
size_t posicao = s.find("valor", <POSICAO>); // primeira ocorrencia
size_t posicao = s.rfind("valor", <POSICAO>); // ultima ocorrencia 
size_t posicao = s.find_first_of("aeiou"); // algum dos chars
size_t posicao = s.find_last_of("aeiou"); // algum dos chars
size_t posicao = s.find_first_not_of("aeiou"); // algum que não é um dos chars
size_t posicao = s.find_last_not_of("aeiou"); // algum que não é um dos chars
bool achou = posicao != string::npos;

// Substring
string s2 = s.substr(INICIO, <TAMANHO>); 

// Comparar
int cmp = compare(<INICIO, TAMANHO>, outra, <INICIO, TAMANHO>);

// Substituir
s.replace(INICIO, TAMANHO, outra); 
s.replace(INICIO, TAMANHO, outra, INICIO, TAMANHO);
s.replace(INICIO, TAMANHO, outra, TAMANHO);
s.replace(INICIO, TAMANHO, QUANTIDADE, CHAR);

// Ler linha
getline(cin, s);
```

## cmath
```cpp
// Funções trigonométricas - em radianos
double angle = 3.14;
double v = sin(angle) + cos(angle) + tan(angle) +
	 acos(1.0) + // [0, pi]
	 asin(1.0) + // [-pi/2,+pi/2] 
	 atan(1.0);  // [-pi/2,+pi/2]
double x,y;
v = atan2(y, x);  // [-pi,+pi]

v = sinh(angle) + cosh(angle) + tanh(angle); // Funções hiperbólicas

double PI = 355.0 / 113.0; //Aproximação de Pi

double base = 3, expoent = 10;
v = sqrt(25);           // Raiz quadrada
v = pow(base, expoent); // 3^10
v = exp(expoent);       // e^10
v = log(1234);          // Logaritmo nepteriano
v = log10(1234);        // Log na base 10

int integer;
double frac = modf(13.4, &integer); // Separa as partes inteira e decimal.

v = log2(1024); // Log na base 2, C++11 apenas.

v = ceil(12.3);  // 13
v = floor(12.3); // 12
v = abs(-3.4);   // 3.4, Módulo

v = fmod(5.3, 2.0); // 1.3, resto fracionário da divisão inteira

// Constantes
v = NAN;
v = HUGE_VAL; // ~= infinito
v = INFINITY; // C++11
```

## C++ Básico
```cpp

// Classe - private por padrão
class Simple { 
  int i_;
public:
  Simple(int i) : i_(i) {}; // Constructor
  ~Simple(); // Destructor
  Simple(const Simple &); // Copy Constructor
  Simple & operator=(const Simple &); // Assignment Operator
  void print_i(); // método
};

// Struct - public por padrão
struct Simple { 
  Simple(int i) : i_(i) {}; // construtor
  void print_i(); // método
private:
  int i_;
};

void Simple::print_i() {
  cout << i_ << endl;
}

// Alocação dinâmica
int * a = new int [3];
delete [] a;

Simplt * s = new Simple;
delete s;

// Parametros:
void foo(int a = 2); // Valor padrão
void foo

// Try catch
try {
  //código
  throw 2;
} catch (RangeError &re) {
  // específica
} catch (int &i) {
  // Exceção 2
} catch (...) {
  // Qualquer exceção
}

vor foo(); // pode lançar qualquer exceção
vor foo() throw(); // não lança exceção
vor foo() throw(int); // só lança exceção int


// Operadores
TIPO operator()(TIPO param1, TIPO param2, ...) // function call
TIPO& operator++() // ++valor
TIPO operator++(int) // valor++
TIPO& operator+=(const TIPO& outro) // +=
TIPO operator+(TIPO primeiro, const TIPO& segundo) // primeiro + segundo
bool operator<(const TIPO& primeiro, const TIPO& segundo) // p < s; único necessário
bool operator==(const TIPO& primeiro, const TIPO& segundo) // p == s; único necessário
const TIPO& operator[](size_t idx) const // var[idx]
```

## limits
```cpp
#include <limits>
// Exemplos para int
numeric_limits<TIPO>::min(); // -2147483648
numeric_limits<TIPO>::max(); // 21477483647
numeric_limits<TIPO>::is_signed(); // 1
numeric_limits<TIPO>::digits(); // 31 bits
numeric_limits<TIPO>::has_infinity; // 0
```

## complex
```cpp
#include <complex>

complex<double> x(REAL, IMAG);
real(x); imag(x); // Pega parte real e imaginária
abs(x); arg(x) // x = abs(x)*e^i*arg(x) rad
norm(x); // Norma de x
conjugate(x); // Conjugado

x = polar(2, 0.5); x = 2*e^i*0.5
```

## Tipos inteiros
```c++
int a;                    // 32 bits,  [-2.147.483.648, 2.147.483.647]
unsigned int a;           // 32 bits,  [0, 4.294.967.295]
long long int a;          // 64 bits,  [-9.223.372.036.854.775.808, 9.223.372.036.854.775.807]
unsigned long long int a; // 64 bits,  [0, 18.446.744.073.709.551.615]
float a;                  // 32 bits,  [1.2E-38, 3.4E+38]
double a;                 // 64 bits,  [2.3E-308, 1.7E+308]
long double a;            // 128 bits, [3.4E-4932, 1.1E+4932]
```

## Java BigInteger
```java
import java.math.BigInteger;

// Constantes
BigInteger.ZERO;
BigInteger.ONE;
BigInteger.TEN;

// Construtores
int base = 10;
BigInteger.valueOf(123456789);
BigInteger.valueOf("123456789");
BigInteger.valueOf("123456789", base);
byte [] binaryRepr; // Representação binária em C2 com bit mais significante à esquerda
BigInteger.valueOf(binaryRepr);

// Operações
BigInteger a, b, c;
int expoent = 0;
c = a.add(b);
c = a.subtract(b);
c = a.multiply(b);
c = a.divide(b);
c = a.mod(b);
c = a.pow(expoent);
BigInteger[] arr = a.divideAndRemainder(b);
BigInteger divider = arr[0], remainder = arr[1];

boolean iguais = a.equals(b);
int comparacao = a.compareTo(b);

//Conversão
int val = a.intValue();
long val = a.longValue();
float val = a.floatValue();
// etc
String strVal = a.toString();
```

## Java IO
```java
import java.util.Scanner;

Scanner s = new Scanner(System.in);

// Leitura de input
int a = s.nextInt();
String str = s.next(); // Próxima palavra, retorna "ab" para input "ab cd"
str = s.nextLine();    // Lê linha
BigInteger bigI = s.nextBigInteger();

//Checagem de fim de input
boolean b;
b = s.hasNextLine();
b = s.hasNextInt();
b = s.hasNext();

// ...
s.close(); // recomendado
```

<br><br>

## Djakstra (custo mínimo)
```python
def djakstra(src, graph):
	costs = [INFINITY for v in graph] #array
	parent = [None for v in graph]    #array
	to_visit = [src]                  #fila
	visited = set([src])              #conjunto
	costs[src] = 0

	while to_visit:
		v = to_visit.pop(0)
		for adj in graph.adjs(v):
			newCost = costs[v] + graph[v][adj]
			if newCost < costs[adj]:
				costs[adj] = newCost
				parent[adj] = v

			if not adj in visited:
				visited.add(adj)
				to_visit.append(adj)

	return costs, parent
```

## Bellman-Ford (custo mínimo, aresta negativa)
```python
def bellman_ford(origin, nodes, edges):
	dist = [float('inf')] * nodes
	pred = [-1] * nodes
	dist[origin], pred[origin] = 0, -2

	for i in range(nodes - 1):
		for edge in edges:
			u, v, weight = edge
			if dist[u] + weight < dist[v]:
				dist[v] = dist[u] + weight
				pred[v] = u

	for edge in edges:
		u, v, weight = edge
		if dist[u] + weight < dist[v]:
			return 'cicle'

	return dist, pred
```
<br><br><br><br><br><br>

## Floyd-Warshall (custo mínimo, todos nós para todos os nós)
```python
def floyd_warshall(nodes, edges):
	dist = [[float('inf')] * nodes for _ in range(nodes)] # N x N
	next = [[-1] * nodes for _ in range(nodes)]
	for u, v, w in edges:
		dist[u][v] = w
		next[u][v] = v

	for k in range(nodes):
		for i in range(nodes):
			for j in range(nodes):
				if dist[i][k] + dist[k][j] < dist[i][j]:
					dist[i][j] = dist[i][k] + dist[k][j]
					next[i][j] = next[i][k]
	return dist, next
```

## BFS
```cpp

bool bfs(T src, T dest){
	queue<T> fila;
	fila.push_back(src);
	set<T> visitados;
	map<T, T> parent;

	while(!fila.empty()){
		T cur = fila.front();

		if(cur == dest){
			return true;
		}

		fila.pop();
		visitados.insert(cur);
		foreach(T adj in cur.adjacencias){
			if(!visitados.contains(adj)){ // Pseudo-code
				fila.push_back(adj);
				parent.insert(make_pair(cur, adj));
				visitados.insert(adj);
			}
		}
	}
	return false;
}
```

<br><br><br><br>

## Prim (árvore geradora mínima)
```python
def findMinEdge(reached_vertices, graph):
	best_edge = None
	for u in reached_vertices:
		for v in adjs(u, graph):
			if not v in reached_vertices:
				candidate = (u,v)
				if not best_edge or cost(candidate) < cost(best_edge):
					best_edge = candidate
	return best_edge


def prim(graph):
	reached_vertices = set([0]) # qualquer vértice como inicial
	selected_edges = []

	while len(reached_vertices) != len(graph): # enquanto não atingir todos os vértices
		minEdge = findMinEdge(reached_vertices, graph)
		selected_edges.append(minEdge)
		reached_vertices = reached_vertices.union(set(minEdge))

	return selected_edges
```

## Ford-Fulkerson (Fluxo máximo)
```cpp

int fordFulkerson(int grapv[V}[V], int s, int t) {
	int u, v;
	int rGraph[V][V]; // Residual graph
	for (u = 0; u < V; v++) 
		for(v = 0; v < V; v++)
			rGraph[u][v] = graph[u][v];
	
	int parent[V]; // armazena caminho, BFS deve usar
	int max_flow = 0;
	
	// Enquanto tem caminho no grafo residual
	while(bfs(rGraph, s, t, parent)) {
		int path_flow = INT_MAX;
		for (v = t; v != s; v = parent[v]) {
			u = parent[v]
			path_flow = min(path_flow, rGraph[u][v]);
		}
		for (v = t; v != s; v = parent[v]) {
			u = parent[v];
			rGraph[u][v] -= path_flow;
			rGraph[v][u] += path_flow;
		}
		
		max_flow += path_flow;
	}
	return max_flow;
}
```

## Longest Common Subsequence
```python
def lcs(a, b):
	X = [0]*(max(len(b), len(a)) + 1)
	Y = [0]*(max(len(b), len(a)) + 1)
	for i in range(len(a), -1, -1):
		for j in range(len(b), -1, -1):
			if i >= len(a) or j >= len(b):
				X[j] = 0
			elif a[i] == b[j]:
				X[j] = 1 + Y[j + 1]
			else:
				X[j] = max(Y[j], X[j + 1])
		Y = [x for x in X] # Copiar X
	return X[0]
```

## Longest Increasing Subsequence
```python
def lis(sequence):
	dp = [1] * len(sequence)
	prev = [1] * len(sequence)
	max_len, best_end = 1, 0

	for i in range(1, len(sequence)):
		dp[i] = 1
		prev[i] = -1

		for j in range(i - 1, -1, -1):
			if dp[j] + 1 > dp[i] and sequence[j] < sequence[i]:
				dp[i] = dp[j] + 1
				prev[i] = j
			if dp[i] > max_len:
				best_end = i
				max_len = dp[i]
	return max_len, best_end, prev
```
<br><br>

## Mochila 01 (Prog Dinâmica)
```python
def mochila01(values, weights, W):
	item_count = len(values)
	table = [[0 for col in range(W+1)] for row in range(item_count+1)] 

	# tabela (items+1) x (W+1)
	# 1a linha e 1a coluna = 0

	for capacity in range(1, W + 1):
		for item in range(1, item_count + 1):
			item_weight = weights[item-1]
			item_value = values[item-1]
			previous_value = table[item-1][capacity]

			if capacity >= item_weight:
				table[item][capacity] = max(table[item-1][capacity - item_weight] + item_value, previous_value)
			else:
				table[item][capacity] = previous_value

	return table[item_count][W]
```

## _Template_ de programação dinâmica
```python
def progDinamica(caso, lookup_table):
	if caso_basico:
		return calcula_caso_basico()
	elif lookup_table.contains(caso):
		return lookup_table[caso]
	else:
		resp_proximo_caso = progDinamica(caso+1, lookup_table)
		return lookup_table[caso] = combina(caso, resp_proximo_caso)
```

## Modular exponentiation ((base ^ exp) % mod)
```cpp
int modular_pow(int base, int exp, int modulus) {
	int result = 1;
	base = base % modulus;
	while (exp > 0) {
		if (exp % 2 == 1) {
			result = (result * base) % modulus;
		}
		exp = exp >> 1;
		base = (base * base) % modulus;
	}
	return result;
}
```

## Sieve of Eratosthenes (todos os primos menores do que N)
```python
def sieve_of_eratosthenes(n):
	primes = []
	used = [0]*(n + 1)
	for i in range(2, n + 1):
		if not used[i]:
			primes.append(i)
			for j in range(i**2, n + 1, i):
				used[j] = 1
	return primes
```

## N-ésimo primo:
```cpp
#include <cmath>
int overestimate_prime(int n) {
	return (int) n*log(n) + n*log(log(n));
}
sieve_of_eratosthenes(overestimate_prime(n))[n - 1];
```

## Contagem:
```
Permutações: (número de ordenações de n elementos em um conjunto de n elementos)
	P(n) = n!
k-Permutações: (número de ordenações de k elementos em um conjunto de n elementos)
	P(n,k) = n! / (n - k)!
Combinações: (número de subconjuntos de tamanho k formados a partir de um conjunto com n elementos)
	C(n,k) = n! / (k! * (n - k)!)
	C(n,k) = C(n-1,k-1) + C(n-1,k)
	C(k,k) = 1
	C(n-k,0) = 1
Partições: (generalizam combinações para r conjuntos)
	Part = n! / (n1! * n2! * ... * nr!)
	
N-ésimo número de Catalan:
	Cn = C(2n,2) / (n + 1)
	~ Quantas maneiras é possível construir fórmulas balanceadas a partir de n pares de parênteses
	~ Quantas triangulações são possíveis em um polígono convexo

Números Eulerianos: (nútmero de permutações de tamanho n com exatamente k sequências ascendentes)
	E(n,k) = (n - k + 1) * E(n-1,k-1) + k * E(n-1,k)
	E(n-k,0) = 1
	E(k+1,k) = 1
	
```

## Fibonacci
```python
def fib_num(n):
	sq5 = sqrt(5)
	G = (1 + sq5)/2.0
	psi = 1 - G
	return (G**n - psi**n)/sq5

```

## Primos
```cpp
int primos[500] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53,  //  0
59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139,// 16
149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, // 34
233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, // 50
331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421, // 66
431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, // 82
523, 541, 547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, // 98
631, 641, 643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, //114
739, 743, 751, 757, 761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829, 839, //130
853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941, 947, 953, //146
967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049,   //162
1051, 1061, 1063, 1069, 1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129,   //176
1151, 1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231,   //189
1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319,	//202
1321, 1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439,	//215
1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 1523,	//228
1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609,	//241
1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709,	//254
1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811,	//267
1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907, 1913,	//280
1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999, 2003, 2011, 2017,	//293
2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113,	//306
2129, 2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237,	//319
2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287, 2293, 2297, 2309, 2311, 2333,	//332
2339, 2341, 2347, 2351, 2357, 2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411,	//345
2417, 2423, 2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 2539,	//358
2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 2621, 2633, 2647, 2657,	//371
2659, 2663, 2671, 2677, 2683, 2687, 2689, 2693, 2699, 2707, 2711, 2713, 2719,	//384
2729, 2731, 2741, 2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819,	//397
2833, 2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903, 2909, 2917, 2927,	//410
2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001, 3011, 3019, 3023, 3037, 3041,	//423
3049, 3061, 3067, 3079, 3083, 3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169,	//436
3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 3259, 3271,	//449
3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343, 3347, 3359, 3361, 3371,	//462
3373, 3389, 3391, 3407, 3413, 3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491,	//475
3499, 3511, 3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571};		//488
```
