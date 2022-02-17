#include <pch.h>
#include "RibbonUI.h"
#include <UxTheme.h>
#include <filesystem>
#include <vector>

#pragma comment(lib, "version.lib")
#pragma comment (lib, "UxTheme.lib")
using namespace GemmaFusion;

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR cmdArgs, int nShowWnd)
{
	//
	// Init application
	//

	// Init logger (debug output)
	AppLogger()->SetLogLevel(0);
	AppLogger()->EnableLine(true);// Output line
	AppLogger()->EnableTime(true);// Output line
	AppLogger()->EnableFunction(true);// Output function name
	AppLogger()->EnableLogging(true);

	APPRESULT_INFORMATION("SymmetryGIS %d", 6);

	// Init
	winrt::init_apartment(winrt::apartment_type::single_threaded);		// Init COM apartment
	SetThreadPerMonitorDpiAwarenessContext();							// This application is HiDPI aware and can handle scaling on its own
	g_ApplicationState.GraphicsProfile = GraphicsProfile::HighQuality;  // Global variable to toggle graphics quality (will be removed in future releases)
	InitializeFrequent();												// Init legacy STEZA "security" layer which decodes commonly used constants used by camera and renderers (DEV MODE)

	ApplicationInstance applicationInstance(hInstance);	// Current application instance with event loop
	Window mainWindow(L"Symmetry GIS", 1300, 700);		// Main window
	
	// Init main spatial projection and transformations
	Epsg epsgStore;
	Crs wgs84 = epsgStore.GetCrs(4326);
	Crs utm32n = epsgStore.GetCrs(3794);
	Crs webMercator = epsgStore.GetCrs(3857);
	CrsTransformation geographicTransform(utm32n, wgs84);
	CrsTransformation projectedTransform(wgs84, utm32n);

	GdiPlusInstance gdiPlusInstance; // Init GDI+ (Required for png, jpg which are primarly used in MapLayerRenderer)
	
	//
	// Init UI 
	// 
	
	// Riborn menu
	RibbonFramework ribbon(mainWindow);
	EmptyLayout ribbonLayout;

	// Canvas (Main data drawing)
	GraphicsInfrastructure gi;
	GraphicsDevice9 deviceD3D9;	// D3D9 graphics device needed for canvas
	GraphicsDevice11 deviceD3D11 = gi.CreateDevice(::Graphics::DebugDevice::None, false); // D3D11 graphics device needed for canvas
	CanvasControl canvas(deviceD3D9, deviceD3D11, mainWindow);	// 2D Canvas for displaying shapes, rasters and lidar
	
	// Status bar
	StatusBar statusBar(mainWindow->GenerateCmdId(), mainWindow);
	statusBar->AddLabel("projection", 350, std::format("🌐 EPSG:{} ({})", utm32n.GetEpsg(), utm32n.GetName()));

	// Prepare main layout for window (binds WM_SIZE handler to the main window)
	StackLayout mainLayout(mainWindow, LayoutType::Vertical);
	ribbon->SetOnViewChange([&]() {
		mainLayout->ModifyLayout(ribbonLayout, UI::SizeType::FixedPixels, ribbon->GetRibbonHeight());
	});
	mainLayout->AddLayout(ribbonLayout); // Top ribbon
	mainLayout->AddControl(canvas);  // Canvas will use rest of the free space
	mainLayout->AddControl(statusBar, SizeType::FixedDips, 25); // Bottom control (statusbar) will have height of 25 dips




	bool enableContext = true;
	
	auto l_enabled = [&]() -> bool
	{
		return !enableContext;
	};

	auto l_invalidate_props = [&]
	{
		//ribbon->InvalidateProperty(IDC_CMD_HOME_BUTTON);
		//ribbon->InvalidateProperty(IDC_CMD_HOME_COLOR);
		//ribbon->InvalidateProperty(IDC_CMD_HOME_CHECKBOX);
		//ribbon->InvalidateProperty(IDC_CMD_HOME_TEXT);
		//ribbon->InvalidateProperty(IDC_CMD_HOME_NUMERIC);
		//ribbon->InvalidateProperty(IDC_CMD_TAB_CONTEXT_GROUP);
	};


	//
	// Init rendering and containers
	//
	Camera camera = canvas->GetCamera(); // Get 3D camera from canvas
	camera->SetZ(-300000.0f);

	auto georeferencer = Core::make_ptr<WorldGeoreferencer>(camera);
	georeferencer->SetBaseScale({ 0.5, -0.5, -0.5 });
	georeferencer->SetBasePoint({ 1741650, 5870027 });

	auto shapeRenderer = Core::make_ptr<ShapeRenderer>(deviceD3D9, camera, georeferencer);
	shapeRenderer->SetDpi(mainWindow->GetDpi());

	ShapeDrawing shapeDrawing;
	shapeDrawing->Initialize({ shapeRenderer });
	shapeDrawing->SetTargetCrs(webMercator);
	auto rasterRenderer = Core::make_ptr<MonoRasterLayerRenderer>(deviceD3D9, camera, georeferencer);
	
	RasterDrawing rasterDrawing;
	rasterDrawing->Initialize({ rasterRenderer });
	rasterDrawing->SetTargetCrs(webMercator);

	MapLayerRenderer mapRenderer(deviceD3D9, camera, georeferencer);
	mapRenderer->EnableTextureFiltering(true);
	mapRenderer->SwapRedBlueColors(true); // 
	mapRenderer->SetTargetCrs(webMercator);

	ShapeLoader shapeLoader;
	shapeLoader->AddLayerRenderer(shapeRenderer);
	shapeLoader->Start();

	LasContainer lasContainer;
	lasContainer->SetTargetCrs(webMercator);

	ShapeLayer lasShapeLayer = lasContainer->GetLasLayer();
	lasShapeLayer->GetStyle().FillSymbol().Show(false); 
	shapeRenderer->CreateLayerObject(lasShapeLayer);

	TmsFetcher basemapFetcher("https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}.png");
	MapLayer basemapObject = mapRenderer->CreateLayerObject(basemapFetcher.as<TileFetcher>());

	// Button click
	//ribbon->SetOnCommandExecute(IDC_CMD_HOME_BUTTON, [&]()
	//{
	//	AppLogger()->WriteLog("IDC_CMD_HOME_BUTTON",0);
	//	//items.push_back(textView->GetSelectedText());
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_TEXT);
	//	//ribbon->InvalidateValue(IDC_CMD_HOME_TEXT);
	//});

	//ribbon->SetOnEnabledUpdate(IDC_CMD_HOME_BUTTON, l_enabled);

	mainWindow->AddEventHandler(WM_CLOSE, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult) {
		applicationInstance->Exit(0); 
		return true; 
	});

	mainWindow->AddEventHandler(WM_DPICHANGED, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult) { 
		shapeRenderer->SetDpi(mainWindow->GetDpi());
		return false; 
	});

	canvas->SetDrawEvent([&](Camera camera)
	{
		deviceD3D9->GetD3D9Device()->SetRenderState(D3DRS_ZENABLE, FALSE);
		mapRenderer->Draw();
		rasterRenderer->Draw();
		shapeRenderer->Update();
		shapeRenderer->Draw();
		return true;
	});

	canvas->SetMouseEvent(MouseEventType::MouseMove, [&](const Mouse& mouse) {
		shapeRenderer->Update(true);
	});

	canvas->SetMouseEvent(MouseEventType::LButtonDown, [&](const Mouse& mouse) {
		canvas->SetCursor(MouseCursor::SizeAll);
	});

	canvas->SetMouseEvent(MouseEventType::LButtonUp, [&](const Mouse& mouse) {
		canvas->SetCursor(MouseCursor::Arrow);
	});

	canvas->SetMouseEvent(MouseEventType::RButtonUp, [&](const Mouse& mouse) {
		ContextMenu canvasContextMenu;
		auto [x, y] = canvas->GetScreenCoordinates(static_cast<int>(mouse.CurrentPosition().x), static_cast<int>(mouse.CurrentPosition().y));
		canvasContextMenu->Open(canvas, x, y);
	});

	canvas->SetMouseWheelEvent([&](const Mouse& mouse, int zDelta) {
		shapeRenderer->Update(true);
	});
	// Load ribbon UI after everything is set
	ribbon->LoadUI();


	mainWindow->Show(SW_SHOW);

	l_invalidate_props();

	mainLayout->Recalculate();
	int returnCode = applicationInstance->Run(0, nullptr);
	shapeLoader->Stop();
	return returnCode;
}